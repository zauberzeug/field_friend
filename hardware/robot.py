import abc
import logging

import numpy as np
import rosys
from rosys.event import Event
from rosys.geometry import Velocity
from rosys.hardware import RobotBrain

from .battery import Battery
from .bms import BmsMessage


class Robot(abc.ABC):
    HOMING_SPEED: float = 8000
    WORKING_SPEED: float = 40000
    MIN_Y: float = 0
    MAX_Y: float = 0.24
    MIN_Z: float = 0
    MAX_Z: float = 0.20
    STEPS_PER_MM_YAXIS: float = 667
    STEPS_PER_MM_ZAXIS: float = 1600

    def __init__(self) -> None:
        self.UPDATED = Event()
        '''the hardware state has been updated'''

        self.VELOCITY_MEASURED = Event()
        '''new velocity measurements are available for processing (argument: list of velocities)'''

        self.log = logging.getLogger('field_friend.robot')

        self.emergency_stop: bool = False

        self.yaxis_end_l: bool = False
        self.yaxis_end_r: bool = False
        self.yaxis_position: int = 0
        self.yaxis_alarm: bool = False
        self.yaxis_idle: bool = False
        self.yaxis_position: int = 0
        self.yaxis_home_position: int = 0

        self.zaxis_end_t: bool = False
        self.zaxis_end_b: bool = False
        self.zaxis_position: int = 0
        self.zaxis_alarm: bool = False
        self.zaxis_idle: bool = False
        self.zaxis_position: int = 0
        self.zaxis_home_position: int = 0

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
        steps = int((linear * 1000) * self.STEPS_PER_MM_ZAXIS)
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
    async def try_reference_yaxis(self) -> None:
        pass

    @abc.abstractmethod
    async def try_reference_zaxis(self) -> None:
        pass

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
            self.log.info("alarm")
            return False
        self.log.info('idle')
        return True

    async def try_reference_yaxis(self) -> None:
        self.log.info('starting homing of Yaxis...')
        await super().try_reference_yaxis()
        self.log.info('driving FAST to left reference point')
        await self.robot_brain.send(
            'y_is_referencing = true;'
            f'yaxis.speed({self.HOMING_SPEED*3});'
        )
        if not await self.check_if_idle_or_alarm_yaxis():
            return
        self.log.info('driving out of reference')
        await self.robot_brain.send(f'yaxis.speed(-{self.HOMING_SPEED});')
        if not await self.check_if_idle_or_alarm_yaxis():
            return
        #self.log.info('driving to left reference')
        # await self.robot_brain.send(f'yaxis.speed({self.HOMING_SPEED});')
        # if not await self.check_if_idle_or_alarm_yaxis():
        #    return
        #self.log.info('driving out of reference and save as HOME')
        await self.robot_brain.send('y_is_referencing = false;')
        # if not await self.check_if_idle_or_alarm_yaxis():
        #    return
        # await rosys.sleep(0.4)
        self.yaxis_home_position = self.yaxis_position
        self.log.info('driving to offset 1cm')
        await self.move_yaxis_to(0.01)
        self.log.info('yaxis referenced')

    async def move_yaxis_to(self, y_position: float, speed: float = 40000) -> None:
        await super().move_yaxis_to()
        self.log.info(f'driving to {y_position}')
        assert self.MIN_Y <= y_position <= self.MAX_Y
        steps = self.linear_to_steps(y_position)
        target_position = self.yaxis_home_position - steps
        await self.robot_brain.send(f'yaxis.position({target_position}, {speed});')
        if not await self.check_if_idle_or_alarm_yaxis():
            return
        self.log.info('reached target')

    async def check_if_idle_or_alarm_zaxis(self) -> bool:
        await rosys.sleep(0.2)
        while not self.zaxis_idle and not self.zaxis_alarm:
            await rosys.sleep(0.5)
        if self.zaxis_alarm:
            self.log.info("alarm")
            return False
        self.log.info('idle')
        return True

    async def try_reference_zaxis(self) -> None:
        self.log.info('starting homing of Zaxis...')
        await super().try_reference_zaxis()
        self.log.info('driving FAST to left reference point')
        await self.robot_brain.send(
            'z_is_referencing = true;'
            f'zaxis.speed({self.HOMING_SPEED*3});'
        )
        if not await self.check_if_idle_or_alarm_zaxis():
            return
        self.log.info('driving out of reference')
        await self.robot_brain.send(f'zaxis.speed(-{self.HOMING_SPEED});')
        if not await self.check_if_idle_or_alarm_zaxis():
            return
        #self.log.info('driving to left reference')
        # await self.robot_brain.send(f'zaxis.speed({self.HOMING_SPEED});')
        # if not await self.check_if_idle_or_alarm_zaxis():
        #    return
        #self.log.info('driving out of reference and save as HOME')
        await self.robot_brain.send('z_is_referencing = false;')
        # if not await self.check_if_idle_or_alarm_zaxis():
        # return
        self.zaxis_home_position = self.zaxis_position
        self.log.info('driving to offset 1cm')
        await self.move_zaxis_to(0.01)
        self.log.info('zaxis referenced')

    async def move_zaxis_to(self, z_position: float, speed: float = 40000) -> None:
        await super().move_zaxis_to()
        self.log.info(f'driving to {z_position}')
        assert self.MIN_Z <= z_position <= self.MAX_Z
        steps = self.depth_to_steps(z_position)
        target_position = self.zaxis_home_position - steps
        await self.robot_brain.send(f'zaxis.position({target_position}, {speed});')
        if not await self.check_if_idle_or_alarm_zaxis():
            return
        self.log.info('reached target')

    async def disable_end_stops(self) -> None:
        await self.robot_brain.send(
            'yend_stops_active = false;'
            'zend_stops_active = false;'
        )

    async def enable_end_stops(self) -> None:
        await self.robot_brain.send(
            'yend_stops_active = true;'
            'zend_stops_active = true;'
        )

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

                # bumper and e-stops
                self.emergency_stop = int(words.pop(0)) == 0 or int(words.pop(0)) == 0

                # yaxis
                self.yaxis_end_l = int(words.pop(0)) == 0
                self.yaxis_end_r = int(words.pop(0)) == 0
                self.yaxis_idle = words.pop(0) == 'true'
                self.yaxis_position = int(words.pop(0))
                self.yaxis_alarm = int(words.pop(0)) == 0

                # zaxis
                self.zaxis_end_t = int(words.pop(0)) == 0
                self.zaxis_end_b = int(words.pop(0)) == 0
                self.zaxis_idle = words.pop(0) == 'true'
                self.zaxis_position = int(words.pop(0))
                self.zaxis_alarm = int(words.pop(0)) == 0

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

    async def drive(self, linear: float, angular: float) -> None:
        await super().drive(linear, angular)

    async def stop_yaxis(self) -> None:
        await super().stop_yaxis()

    async def stop_zaxis(self) -> None:
        await super().stop_zaxis()

    # TODO simulate battery and emergency stop
