from ..hardware.imu import Eulerangle
from field_friend.hardware import FieldFriend
import logging
import rosys


class Falling:
    
    def __init__(self, field_friend: FieldFriend) -> None:
        self.log = logging.getLogger('field_friend.falling')
        self.field_friend = field_friend
        self.has_stopped = False
        self.rescue_in_progress_val = False
    
    def reset(self)->None:
        self.has_stopped = False
        self.rescue_in_progress_val = False
    

    def rescue_in_progress(self)->None:
        self.rescue_in_progress_val = True
        self.has_stopped = False


        
        
class FallingHardware(Falling):

    def __init__(self,field_friend: FieldFriend) -> None:
        self.field_friend = field_friend
        if self.field_friend.config['falling']['version'] == 'active':
            self.roll_limit = self.field_friend.config['falling']['roll']
            self.pitch_limit = self.field_friend.config['falling']['pitch']
        self.field_friend.imu.ROBOT_ANGLES.register(self.is_falling)
        super().__init__( field_friend= field_friend)

    async def is_falling(self,eulerangles: Eulerangle)->None:
        if self.rescue_in_progress_val:
            if (eulerangles.roll> self.roll_limit+5 or
            eulerangles.roll< -self.roll_limit-5):
                if not self.has_stopped:
                    self.log.info(f'robot exeeds roll emgergency limit with {eulerangles.roll}')
                    rosys.notify('Robot exeeded roll emgergency limit, stopping','warning')
                    self.has_stopped = True
                    await self.field_friend.estop.set_soft_estop(True)

            if (eulerangles.roll> self.pitch_limit+5 or
            eulerangles.roll< -self.pitch_limit-5):
                if not self.has_stopped:
                    self.log.info(f'robot exeeds pitch emergency limit with {eulerangles.roll}')
                    rosys.notify('Robot exeeded roll emergency limit, stopping','warning')
                    self.has_stopped = True
                    await self.field_friend.estop.set_soft_estop(True)


        else:
            if (eulerangles.roll> self.roll_limit or
                eulerangles.roll< -self.roll_limit):
                if not self.has_stopped:
                    self.log.info(f'robot exeeds roll limit with {eulerangles.roll}')
                    rosys.notify('Robot exeeded roll limit, stopping','warning')
                    self.has_stopped = True
                    await self.field_friend.estop.set_soft_estop(True)

            
            if (eulerangles.pitch> self.pitch_limit or
                eulerangles.pitch< -self.pitch_limit):
                if not self.has_stopped:
                    self.log.info(f'robot exeeds pitch limit with {eulerangles.pitch} of {self.pitch_limit}')
                    rosys.notify('Robot exeeded pitch limit, stopping','warning')
                    self.log.info(f'the limit is:{self.pitch_limit}')
                    self.has_stopped = True
                    await self.field_friend.estop.set_soft_estop(True)


    

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
            self.log.info(f'robot exeeds roll limit with {eulerangles.roll}')
            rosys.notify('Robot exeeded roll limit, stopping','warning')
            self.has_stopped = True
            await self.field_friend.estop.set_soft_estop(True)
        
        if (eulerangles.pitch> self.pitch_limit or
            eulerangles.pitch< -self.pitch_limit):
            self.log.info(f'robot exeeds pitch limit with {eulerangles.roll}')
            rosys.notify('Robot exeeded pitch limit, stopping','warning')
            self.has_stopped = True
            await self.field_friend.estop.set_soft_estop(True)