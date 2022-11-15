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
        self.zaxis_end_t: bool = False
        self.zaxis_end_b: bool = False
        self.zaxis_position: int = 0
        self.zaxis_alarm: bool = False
        self.zaxis_idle: bool = False
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

    async def update(self) -> None:
        self.UPDATED.emit()

    async def stop(self) -> None:
        await self.drive(0, 0)

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

    async def stop_zaxis(self) -> None:
        await super().stop_zaxis()

    async def check_if_idle_or_alarm(self) -> bool:
        # Give time for flag to flip
        await rosys.sleep(0.2)
        while not self.yaxis_idle and not self.yaxis_alarm:
            await rosys.sleep(0.4)
        if self.yaxis_alarm:
            self.log.info("alarm")
            return False
        self.log.info('idle')
        return True

    async def try_reference_yaxis(self, speed: int = 8000) -> None:
        self.log.info('starting homing')
        await super().try_reference_yaxis()
        self.log.info('driving to left reference point')
        await self.robot_brain.send(f'yaxis.speed({speed});')
        if not await self.check_if_idle_or_alarm():
            return
        self.log.info('driving out of reference')
        await self.robot_brain.send(
            'y_is_referencing = true;'
            'yaxis.position = 0;'
            f'yaxis.speed(-{speed});'
        )
        if not await self.check_if_idle_or_alarm():
            return
        await self.robot_brain.send(
            'y_is_referencing = false;'
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

                # zaxis
                self.zaxis_end_t = int(words.pop(0)) == 0
                self.zaxis_end_b = int(words.pop(0)) == 0

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
