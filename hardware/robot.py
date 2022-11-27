import abc
import logging
from typing import Callable, Optional

import numpy as np
import rosys
from rosys.event import Event
from rosys.geometry import Velocity
from rosys.hardware import RobotBrain
from rosys.helpers import ramp

from .battery import Battery
from .bms import BmsMessage


class Robot(abc.ABC):
    HOMING_SPEED: float = 8000
    WORKING_SPEED: float = 80000
    MIN_Y: float = -0.12
    MAX_Y: float = 0.12
    AXIS_OFFSET_Y = 0.123
    MIN_Z: float = -0.197
    MAX_Z: float = -0.003
    STEPS_PER_MM_YAXIS: float = 666.67
    STEPS_PER_MM_ZAXIS: float = 1600
    AXIS_OFFSET_X = 0.2915

    def __init__(self) -> None:
        self.UPDATED = Event()
        '''the hardware state has been updated'''

        self.VELOCITY_MEASURED = Event()
        '''new velocity measurements are available for processing (argument: list of velocities)'''

        self.ESTOP_TRIGGERED = Event()
        '''estop was triggered'''

        self.log = logging.getLogger('field_friend.robot')

        self.yaxis_end_l: bool = False
        self.yaxis_end_r: bool = False
        self.yaxis_position: int = 0
        self.yaxis_alarm: bool = False
        self.yaxis_idle: bool = False
        self.yaxis_is_referenced: bool = False
        self.yaxis_home_position: int = 0

        self.zaxis_end_t: bool = False
        self.zaxis_end_b: bool = False
        self.zaxis_position: int = 0
        self.zaxis_alarm: bool = False
        self.zaxis_idle: bool = False
        self.zaxis_is_referenced: bool = False
        self.zaxis_home_position: int = 0

        self.end_stops_active: bool = True
        self.emergency_stop: bool = False
        self.battery: Battery = Battery()

        self.linear_target_speed: float = 0
        self.angular_target_speed:  float = 0

        rosys.on_repeat(self.update, 0.01)
        rosys.on_shutdown(self.stop)

    @property
    def is_real(self) -> bool:
        return isinstance(self, RobotHardware)

    @property
    def is_simulation(self) -> bool:
        return isinstance(self, RobotSimulation)

    def depth_to_steps(self, depth: float) -> int:
        steps = int((depth * 1000) * self.STEPS_PER_MM_ZAXIS)
        return steps

    def steps_to_depth(self, steps: int) -> float:
        depth = (steps * (1 / self.STEPS_PER_MM_ZAXIS)) / 1000
        return depth

    def linear_to_steps(self, linear: float) -> int:
        steps = int((linear * 1000) * self.STEPS_PER_MM_YAXIS)
        return steps

    def steps_to_linear(self, steps: int) -> float:
        linear = (steps * (1 / self.STEPS_PER_MM_YAXIS)) / 1000
        return linear

    async def update(self) -> None:
        self.UPDATED.emit()

    async def stop(self) -> None:
        await self.drive(0, 0)
        await self.stop_yaxis()
        await self.stop_zaxis()

    async def start_homing(self) -> bool:
        try:
            if not self.end_stops_active:
                self.log.warning('end stops not activated')
                return False
            if self.yaxis_end_l or self.yaxis_end_r or self.zaxis_end_b or self.zaxis_end_t:
                self.log.warning('remove from end stops to start homing')
                return False
            if not await self.try_reference_zaxis():
                return False
            if not await self.try_reference_yaxis():
                return False
            return True
        finally:
            await self.stop()

    @abc.abstractmethod
    async def drive(self, linear: float, angular: float) -> None:
        self.linear_target_speed = linear
        self.angular_target_speed = angular

    @abc.abstractmethod
    async def stop_yaxis(self) -> None:
        pass

    @abc.abstractmethod
    async def stop_zaxis(self) -> None:
        pass

    @abc.abstractmethod
    async def try_reference_yaxis(self) -> bool:
        return True

    @abc.abstractmethod
    async def try_reference_zaxis(self) -> bool:
        return True

    @abc.abstractmethod
    async def move_yaxis_to(self) -> None:
        pass

    @abc.abstractmethod
    async def move_zaxis_to(self) -> None:
        pass


class RobotHardware(Robot):

    def __init__(self, robot_brain: RobotBrain) -> None:
        super().__init__()

        self.robot_brain = robot_brain

        self.last_battery_request: float = 0
        self.last_can_send_error: float = 0
        self.last_logging = 0

    async def drive(self, linear: float, angular: float) -> None:
        await super().drive(linear, angular)
        await self.robot_brain.send(f'wheels.speed({linear}, {angular})')

    async def stop_yaxis(self) -> None:
        await super().stop_yaxis()
        await self.robot_brain.send('yaxis.stop()')

    async def stop_zaxis(self) -> None:
        await super().stop_zaxis()
        await self.robot_brain.send('zaxis.stop()')

    async def check_if_idle_or_alarm_yaxis(self) -> bool:
        await rosys.sleep(0.2)
        while not self.yaxis_idle and not self.yaxis_alarm:
            await rosys.sleep(0.4)
        if self.yaxis_alarm:
            self.log.info("yaxis alarm")
            return False
        return True

    async def try_reference_yaxis(self) -> bool:
        if not await super().try_reference_yaxis():
            return False

        try:
            await rosys.sleep(0.2)
            if self.yaxis_end_l or self.yaxis_end_r:
                self.log.info('yaxis is in end stops')
                return False
            self.log.info('starting homing of yaxis...')
            await self.robot_brain.send(
                'y_is_referencing = true;'
                f'yaxis.speed({self.HOMING_SPEED*3});'
            )
            if not await self.check_if_idle_or_alarm_yaxis():
                return False
            await self.robot_brain.send(f'yaxis.speed(-{self.HOMING_SPEED*2});')
            if not await self.check_if_idle_or_alarm_yaxis():
                return False
            await self.robot_brain.send(f'yaxis.speed({self.HOMING_SPEED});')
            if not await self.check_if_idle_or_alarm_yaxis():
                return False
            await self.robot_brain.send(f'yaxis.speed(-{self.HOMING_SPEED});')
            if not await self.check_if_idle_or_alarm_yaxis():
                return False
            await self.robot_brain.send('y_is_referencing = false;')
            await rosys.sleep(0.2)
            self.yaxis_home_position = self.yaxis_position
            self.yaxis_is_referenced = True
            self.log.info('yaxis referenced')
            await self.move_yaxis_to(self.MAX_Y, speed=self.HOMING_SPEED*3)
            return True
        finally:
            await self.stop()

    async def move_yaxis_to(self, y_world_position: float, speed: float = 80000) -> None:
        await super().move_yaxis_to()
        if not self.yaxis_is_referenced:
            self.log.info('yaxis ist not referenced')
            return
        if self.yaxis_end_l or self.yaxis_end_r:
            self.log.info('yaxis is in end stops')
            return
        assert speed <= self.WORKING_SPEED
        assert self.MIN_Y <= y_world_position <= self.MAX_Y
        y_axis_position: float = y_world_position - self.AXIS_OFFSET_Y
        steps = self.linear_to_steps(y_axis_position)
        target_position = self.yaxis_home_position + steps
        await self.robot_brain.send(f'yaxis.position({target_position}, {speed}, 160000);')
        if not await self.check_if_idle_or_alarm_yaxis():
            return
        self.log.info('yaxis reached target')

    async def check_if_idle_or_alarm_zaxis(self) -> bool:
        await rosys.sleep(0.2)
        while not self.zaxis_idle and not self.zaxis_alarm:
            await rosys.sleep(0.5)
        if self.zaxis_alarm:
            self.log.info("zaxis alarm")
            return False
        return True

    async def try_reference_zaxis(self) -> bool:
        if not await super().try_reference_zaxis():
            return False
        try:
            await rosys.sleep(0.2)
            if self.zaxis_end_t or self.zaxis_end_b:
                self.log.info('zaxis is in end stops')
                return False
            self.log.info('starting homing of zaxis...')
            await self.robot_brain.send(
                'z_is_referencing = true;'
                f'zaxis.speed({self.HOMING_SPEED*4});'
            )
            if not await self.check_if_idle_or_alarm_zaxis():
                return False
            await self.robot_brain.send(f'zaxis.speed(-{self.HOMING_SPEED*3});')
            if not await self.check_if_idle_or_alarm_zaxis():
                return False
            await self.robot_brain.send(f'zaxis.speed({self.HOMING_SPEED});')
            if not await self.check_if_idle_or_alarm_zaxis():
                return False
            await self.robot_brain.send(f'zaxis.speed(-{self.HOMING_SPEED});')
            if not await self.check_if_idle_or_alarm_zaxis():
                return False
            await self.robot_brain.send('z_is_referencing = false;')
            await rosys.sleep(0.2)
            self.zaxis_home_position = self.zaxis_position
            self.zaxis_is_referenced = True
            await self.move_zaxis_to(self.MAX_Z, speed=self.HOMING_SPEED*3)
            self.log.info('zaxis referenced')
            return True
        finally:
            await self.stop()

    async def move_zaxis_to(self, z_world_position: float, speed: float = 160000) -> None:
        await super().move_zaxis_to()
        if not self.zaxis_is_referenced:
            self.log.info('zaxis ist not referenced')
            return
        if self.zaxis_end_t or self.zaxis_end_b:
            self.log.info('zaxis is in end stops')
            return
        assert speed <= self.WORKING_SPEED
        assert self.MIN_Z <= z_world_position <= self.MAX_Z
        steps = self.depth_to_steps(z_world_position)
        target_position = self.zaxis_home_position + steps
        await self.robot_brain.send(f'zaxis.position({target_position}, {speed}, 160000);')
        if not await self.check_if_idle_or_alarm_zaxis():
            return
        self.log.info('zaxis reached target')

    async def enable_end_stops(self, value: bool) -> None:
        await self.robot_brain.send(
            f'yend_stops_active = {str(value).lower()};'
            f'zend_stops_active = {str(value).lower()};'
        )
        self.end_stops_active = value
        self.log.info(f'end stops active = {value}')

    async def update(self) -> None:
        velocities: list[Velocity] = []

        for time, line in await self.robot_brain.read_lines():
            words = line.split()

            # core
            if words[0] == 'core':
                words.pop(0)
                words.pop(0)

                # odometry
                velocities.append(Velocity(linear=float(words.pop(0)), angular=float(words.pop(0)), time=time))

                # e-stops
                self.emergency_stop = int(words.pop(0)) == 0 or int(words.pop(0)) == 0
                if self.emergency_stop:
                    self.ESTOP_TRIGGERED.emit()

                # yaxis
                self.yaxis_end_l = int(words.pop(0)) == 0
                self.yaxis_end_r = int(words.pop(0)) == 0
                self.yaxis_idle = words.pop(0) == 'true'
                self.yaxis_position = int(words.pop(0))
                self.yaxis_alarm = int(words.pop(0)) == 0
                if self.yaxis_alarm:
                    self.yaxis_is_referenced = False

                # zaxis
                self.zaxis_end_t = int(words.pop(0)) == 0
                self.zaxis_end_b = int(words.pop(0)) == 0
                self.zaxis_idle = words.pop(0) == 'true'
                self.zaxis_position = int(words.pop(0))
                self.zaxis_alarm = int(words.pop(0)) == 0
                if self.zaxis_alarm:
                    self.zaxis_is_referenced = False

            # battery
            if line.startswith('expander: bms'):
                msg = BmsMessage([int(w, 16) for w in line.split()[2:]])
                msg.check()
                result = msg.interpret()
                self.battery.percentage = result.get('capacity percent')
                self.battery.voltage = result.get('total voltage')
                self.battery.current = result.get('current')
                self.battery.temperature = np.mean(result['temperatures']) if 'temperatures' in result else None
                self.battery.is_charging = (self.battery.current or 0) > 0
                self.battery.last_update = rosys.time()
                if rosys.time() > self.last_logging + 5 * 60:  # only log every five minutes
                    self.last_logging = rosys.time()
                    self.log.info(f'battery: {self.battery.short_string}')
            if 'could not send CAN message' in line:
                self.last_can_send_error = rosys.time()
            if 'CAN timeout for msg id' in line and '(attempt 3/3)' in line:
                self.last_can_send_error = rosys.time()

        self.VELOCITY_MEASURED.emit(velocities)

        # battery requests
        battery_interval = 1.0 if rosys.time() > self.battery.last_update + 5.0 else 5.0
        if rosys.time() > self.last_battery_request + battery_interval:
            await self.robot_brain.send('bms.send(0xdd, 0xa5, 0x03, 0x00, 0xff, 0xfd, 0x77)')
            self.last_battery_request = rosys.time()

        await super().update()


class RobotSimulation(Robot):

    def __init__(self) -> None:
        super().__init__()

        self.pose: rosys.geometry.Pose = rosys.geometry.Pose()
        self.linear_velocity: float = 0
        self.angular_velocity: float = 0
        self.is_charging: bool = False
        self.last_update: Optional[float] = None
        self.yaxis_velocity: float = 0.0
        self.zaxis_velocity: float = 0.0
        self.yaxis_target: Optional[float] = None
        self.zaxis_target: Optional[float] = None

    async def drive(self, linear: float, angular: float) -> None:
        if self.emergency_stop:
            return
        await super().drive(linear, angular)
        self.linear_velocity = self.linear_target_speed
        self.angular_velocity = self.angular_target_speed

    async def stop_yaxis(self) -> None:
        await super().stop_yaxis()
        self.yaxis_velocity = 0
        self.yaxis_target = None

    async def stop_zaxis(self) -> None:
        await super().stop_zaxis()
        self.zaxis_velocity = 0
        self.zaxis_target = None

    # TODO simulate battery and emergency stop

    async def try_reference_yaxis(self) -> None:
        if not await super().try_reference_yaxis():
            return False
        self.yaxis_position = 0
        self.yaxis_is_referenced = True
        return True

    async def try_reference_zaxis(self) -> None:
        if not await super().try_reference_zaxis():
            return False
        self.zaxis_position = 0
        self.zaxis_is_referenced = True
        return True

    async def move_yaxis_to(self, y_world_position: float, speed: float = 80000) -> None:
        await super().move_yaxis_to()
        if not self.yaxis_is_referenced:
            self.log.info('yaxis ist not referenced')
            return
        if self.yaxis_end_l or self.yaxis_end_r or self.emergency_stop:
            self.log.info('yaxis is in end stops')
            return
        assert speed <= self.WORKING_SPEED
        assert self.MIN_Y <= y_world_position <= self.MAX_Y
        y_axis_position: float = y_world_position - self.AXIS_OFFSET_Y
        steps = self.linear_to_steps(y_axis_position)
        self.yaxis_target = self.yaxis_home_position + steps
        self.yaxis_velocity = speed

    async def move_zaxis_to(self, z_world_position: float, speed: float = 80000) -> None:
        await super().move_zaxis_to()
        if not self.zaxis_is_referenced:
            self.log.info('zaxis ist not referenced')
            return
        if self.zaxis_end_t or self.zaxis_end_b or self.emergency_stop:
            self.log.info('zaxis is in end stops')
            return
        assert speed <= self.WORKING_SPEED
        assert self.MIN_Z <= z_world_position <= self.MAX_Z
        steps = self.depth_to_steps(z_world_position)
        self.zaxis_target = self.zaxis_home_position + steps
        self.zaxis_velocity = speed

    async def update(self) -> None:
        await super().update()

        # time
        dt = rosys.time() - self.last_update if self.last_update is not None else 0
        self.last_update = rosys.time()

        # odometry
        self.pose += rosys.geometry.PoseStep(linear=dt*self.linear_velocity,
                                             angular=dt*self.angular_velocity, time=rosys.time())
        velocity = Velocity(linear=self.linear_velocity, angular=self.angular_velocity, time=rosys.time())
        self.VELOCITY_MEASURED.emit([velocity])

        # yaxis
        self.yaxis_position += dt * self.yaxis_velocity
        if self.yaxis_target is not None:
            if (self.yaxis_velocity > 0) == (self.yaxis_position > self.yaxis_target):
                self.yaxis_position = self.yaxis_target
                self.yaxis_target = None
                self.yaxis_velocity = 0

        # zaxis
        self.zaxis_position += dt * self.zaxis_velocity
        if self.zaxis_target is not None:
            if (self.zaxis_velocity > 0) == (self.zaxis_position > self.zaxis_target):
                self.zaxis_position = self.zaxis_target
                self.zaxis_target = None
                self.zaxis_velocity = 0

        # battery
        self.battery.is_charging = self.is_charging
        self.battery.voltage = 25.0 + np.sin(0.01 * rosys.time())
        self.battery.percentage = ramp(self.battery.voltage, 24.0, 26.0, 30.0, 70.0)
        self.battery.current = 1.0 if self.battery.is_charging else -0.7
        self.battery.temperature = 20.0 + np.sin(0.01 * rosys.time())
        self.battery.last_update = rosys.time()
