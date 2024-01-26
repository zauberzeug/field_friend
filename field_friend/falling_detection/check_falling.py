from ..hardware.imu import Eulerangle
from field_friend.hardware import FieldFriend


class Falling:
    
    def __init__(self, field_friend = FieldFriend) -> None:

        self.field_friend = field_friend

        
        
class FallingHardware(Falling):

    def __init__(self,field_friend: FieldFriend) -> None:
        self.field_friend = field_friend
        if self.field_friend.config['falling']['version'] == 'calibrated':
            self.roll_limit = self.field_friend.config['falling']['roll']
            self.pitch_limit = self.field_friend.config['falling']['pitch']

        self.field_friend.imu.ROBOT_ANGLES.register(self.is_falling)
        super().__init__( field_friend= field_friend)

    async def is_falling(self,eulerangles: Eulerangle)->None:
        if (eulerangles.roll> self.roll_limit or
            eulerangles.roll< -self.roll_limit):
            await self.field_friend.estop.set_soft_estop(True)
        
        if (eulerangles.pitch> self.pitch_limit or
            eulerangles.pitch< -self.pitch_limit):
            await self.field_friend.estop.set_soft_estop(True)

    #TODO ist es übernaubt nötig?? in der Konfig für den Rooboter bestimmen
    def zeroing(self,roll: float, pitch: float) -> None:
        self.offset_roll = roll
        self.offset_pitch = pitch

class FallingSimulation(Falling):
    def __init__(self,field_friend: FieldFriend) -> None:
        self.roll_limit:float = 30
        self.pitch_limit:float  = 30 
        self.field_friend = field_friend
        
        self.field_friend.imu.ROBOT_ANGLES.register(self.is_falling)
        super().__init__(field_friend= field_friend)
    
    async def is_falling(self,eulerangles: Eulerangle)->None:
        if (eulerangles.roll> self.roll_limit or
            eulerangles.roll< -self.roll_limit):
            await self.field_friend.estop.set_soft_estop(True)
        
        if (eulerangles.pitch> self.pitch_limit or
            eulerangles.pitch< -self.pitch_limit):
            await self.field_friend.estop.set_soft_estop(True)