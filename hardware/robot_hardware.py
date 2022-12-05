import numpy as np
import rosys
from rosys.geometry import Velocity
from rosys.hardware import RobotBrain

from .bms import BmsMessage
from .robot import Robot


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
            rosys.notify('yaxis ist not referenced')
            return
        if self.yaxis_end_l or self.yaxis_end_r:
            self.log.info('yaxis is in end stops')
            rosys.notify('yaxis is in end stops')
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
            rosys.notify('zaxis ist not referenced')
            return
        if self.zaxis_end_t or self.zaxis_end_b:
            self.log.info('zaxis is in end stops')
            rosys.notify('zaxis is in end stops')
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
