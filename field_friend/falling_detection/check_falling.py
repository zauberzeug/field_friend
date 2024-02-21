from field_friend.hardware import FieldFriend
import logging
import rosys
from rosys.geometry import Rotation
import numpy as np


class Falling:

    def __init__(self, field_friend: FieldFriend) -> None:
        self.log = logging.getLogger('field_friend.falling')
        self.field_friend = field_friend
        self.has_stopped = False
        self.rescue_in_progress_val = False
        self.rescue_stop = False

    async def reset_emergency(self):
        self.has_stopped = False
        self.rescue_stop = False
        self.rescue_in_progress_val = False
        await self.field_friend.estop.set_soft_estop(False)

    def reset(self) -> None:
        self.has_stopped = False
        self.rescue_in_progress_val = False

    def rescue_in_progress(self) -> None:
        self.rescue_in_progress_val = True
        self.has_stopped = False

    async def is_falling(self, euler=tuple[float, float, float]) -> None:
        roll, pitch, yaw = euler
        roll = abs(np.degrees(roll))
        pitch = abs(np.degrees(pitch))
        if self.rescue_in_progress_val:
            if (roll > self.roll_limit+5):
                if not self.rescue_stop:
                    self.log.info(f'robot exeeds roll emgergency limit with {roll}')
                    rosys.notify('Robot exeeded roll emgergency limit, stopping', 'warning')
                    self.has_stopped = False
                    self.rescue_stop = True
                    await self.field_friend.estop.set_soft_estop(True)

            if (pitch > self.pitch_limit+5):
                if not self.rescue_stop:
                    self.log.info(f'robot exeeds pitch emergency limit with {pitch}')
                    rosys.notify('Robot exeeded pitch emergency limit, stopping', 'warning')
                    self.has_stopped = False
                    self.rescue_stop = True
                    await self.field_friend.estop.set_soft_estop(True)

        else:
            if (roll > self.roll_limit):
                if not self.has_stopped or self.rescue_stop:
                    self.log.info(f'robot exeeds roll limit with {roll}')
                    rosys.notify('Robot exeeded roll limit, stopping', 'warning')
                    self.has_stopped = True
                    await self.field_friend.estop.set_soft_estop(True)

            if (pitch > self.pitch_limit):
                if not self.has_stopped or self.rescue_stop:
                    self.log.info(f'robot exeeds pitch limit with {pitch}')
                    rosys.notify('Robot exeeded pitch limit, stopping', 'warning')
                    self.log.info(f'the limit is:{self.pitch_limit}')
                    self.has_stopped = True
                    await self.field_friend.estop.set_soft_estop(True)


class FallingHardware(Falling):

    def __init__(self, field_friend: FieldFriend) -> None:
        self.field_friend = field_friend
        self.roll_limit: float
        self.pitch_limit: float
        if self.field_friend.config['params']['falling_detection'] == 'active':
            self.roll_limit = self.field_friend.config['params']['roll_limit']
            self.pitch_limit = self.field_friend.config['params']['pitch_limit']
            self.field_friend.imu.NEW_MEASUREMENT.register(self.is_falling)
        super().__init__(field_friend=field_friend)


class FallingSimulation(Falling):
    def __init__(self, field_friend: FieldFriend) -> None:
        self.roll_limit: float = 30
        self.pitch_limit: float = 30
        self.field_friend = field_friend
        self.field_friend.imu.NEW_MEASUREMENT.register(self.is_falling)
        super().__init__(field_friend=field_friend)
