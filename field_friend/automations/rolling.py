import logging
from typing import Any
from field_friend.hardware import (IMU, FieldFriend)

import rosys

class Rolling(rosys.persistence.PersistentModule):
    def __init__(self,imu : IMU, field_friend: FieldFriend) -> None:
        super().__init__()
        self.log = logging.getLogger('field_friend.rolling')
        self.imu = imu
        self.field_friend = field_friend
        self.left_roll_limit = -30
        self.right_roll_limit = 30
        self.forward_pitch_limit = -30
        self.backward_pitch_limit = 30
        self.offste_roll = 0
        self.offset_pitch = 0

    def backup(self) -> dict[str, Any]:
        return {
            'left_roll_limit': self.left_roll_limit,
            'right_roll_limit':  self.right_roll_limit,
            'forward_pitch_limit': self.forward_pitch_limit,
            'backward_pitch_limit': self.backward_pitch_limit,
            'offset_roll': self.offste_roll,
            'offste_pitch': self.offset_pitch
        }
    
    def restore(self, data: dict[str, Any]) -> None:
        self.left_roll_limit = data.get('left_roll_limit', self.left_roll_limit)
        self.right_roll_limit = data.get('right_roll_limit', self.right_roll_limit)
        self.forward_pitch_limit = data.get('forward_pitch_limit', self.forward_pitch_limit)
        self.backward_pitch_limit = data.get('backward_pitch_limit', self.backward_pitch_limit)
        self.offste_roll = data.get('offset_roll', self.offste_roll)
        self.offset_pitch = data.get('offset_pitch', self.offset_pitch)

    def set_roll_angle_left(self, left : float) -> None:
        self.left_roll_limit = left

    def set_roll_angle_rigth(self, rigth : float) -> None:
        self.right_roll_limit = rigth
    
    def set_pitch_angle_forward(self, forward: float) -> None:
        self.forward_pitch_limit = forward

    def set_pitch_angle_backward(self, backward: float) -> None:
        self.backward_pitch_limit = backward

    #TODO ist es übernaubt nötig??
    def zeroing(self,roll: float, pitch: float) -> None:
        self.offset_roll = roll
        self.offset_pitch = pitch
    
    async def is_rolled(self) -> None:
        if (self.imu.roll > self.right_roll_limit or
            self.imu.roll < self.left_roll_limit):
            await self.field_friend.estop.set_soft_estop(True)

    async def is_pitched(self) -> None:
        if (self.imu.pitch > self.backward_pitch_limit or
            self.imu.pitch < self.forward_pitch_limit):
            await self.field_friend.estop.set_soft_estop(True)