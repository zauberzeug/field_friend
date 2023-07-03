import logging

import rosys
from rosys.driving import Driver
from rosys.geometry import Point

from ..hardware import FieldFriend


class Puncher:
    def __init__(self, field_friend: FieldFriend, driver: Driver) -> None:
        self.field_friend = field_friend
        self.driver = driver
        self.log = logging.getLogger('field_friend.puncher')

    async def try_home(self) -> bool:
        try:
            if self.field_friend.estop.active:
                rosys.notify('Estop active, relaese first', 'error')
                return False
            if not await self.field_friend.y_axis.try_reference():
                return False
            await rosys.sleep(0.2)
            if not await self.field_friend.z_axis.try_reference():
                return False
            return True
        except Exception as e:
            raise Exception('homing failed') from e
        finally:
            await self.field_friend.y_axis.stop()
            await self.field_friend.z_axis.stop()

    async def drive_to_punch(self, local_target_x: float) -> None:
        self.log.info(f'Driving to punch at {local_target_x}...')
        if local_target_x < self.driver.odometer.prediction.x + self.field_friend.WORK_X:
            self.log.info('Target is behind')
            return
        axis_distance = local_target_x - self.field_friend.WORK_X
        local_target = Point(x=axis_distance, y=0)
        world_target = self.driver.odometer.prediction.transform(local_target)
        await self.driver.drive_to(world_target)

    async def punch(self, y: float, depth: float) -> None:
        try:
            if not self.field_friend.y_axis.is_referenced or not self.field_friend.z_axis.is_referenced:
                rosys.notify('axis are not referenced')
                return
            await self.field_friend.y_axis.move_to(y)
            await self.field_friend.z_axis.move_to(depth)
            await self.field_friend.z_axis.return_to_reference()
        except Exception as e:
            raise Exception('punching failed') from e
        finally:
            await self.field_friend.y_axis.stop()
            await self.field_friend.z_axis.stop()

    async def clear_view(self) -> None:
        self.log.info('Clearing view...')
        y = self.field_friend.y_axis.MIN_POSITION if self.field_friend.y_axis.position <= 0 else self.field_friend.y_axis.MAX_POSITION
        await self.field_friend.y_axis.move_to(y)
        await self.field_friend.y_axis.stop()

    async def drive_and_punch(self, x: float, y: float, depth: float = 0.05) -> None:
        await self.drive_to_punch(x)
        await self.punch(y, depth)
        await self.clear_view()
