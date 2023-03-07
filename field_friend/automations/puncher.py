import rosys
from rosys.driving import Driver
from rosys.geometry import Point

from ..hardware import FieldFriend


class Puncher:
    def __init__(self, field_friend: FieldFriend, driver: Driver) -> None:
        self.field_friend = field_friend
        self.driver = driver

    async def home(self) -> None:
        try:
            if not await self.field_friend.y_axis.try_reference():
                return False
            if not await self.field_friend.z_axis.try_reference():
                return False
        except Exception as e:
            raise Exception('homing failed') from e
        finally:
            self.field_friend.y_axis.stop()
            self.field_friend.z_axis.stop()

    async def drive_to_punch(self, target_distance_x: float) -> None:
        min_drive_distance = 0.1
        axis_distance = max(target_distance_x - self.field_friend.y_axis.AXIS_OFFSET_X, min_drive_distance)
        target_point = Point(x=axis_distance, y=0)
        await self.driver.drive_to(target_point)
        await rosys.sleep(0.2)
        await self.field_friend.wheels.stop()

    async def punch(self, y: float, depth: float = None, speed: float = None) -> None:
        try:
            if not self.field_friend.y_axis.is_referenced or not self.field_friend.z_axis.is_referenced:
                rosys.notify('axis are not referenced')
                return
            if speed == None:
                speed = self.field_friend.z_axis.Z_AXIS_MAX_SPEED
                speed = self.field_friend.y_axis.Y_AXIS_MAX_SPEED
            await self.field_friend.y_axis.move_to(y, speed)
            if depth == None:
                depth = self.field_friend.z_axis.drill_depth
            await self.field_friend.z_axis.move_to(depth, speed)
            await self.field_friend.z_axis.move_to(self.field_friend.z_axis.MAX_Z, speed)
        except Exception as e:
            raise Exception('punching failed') from e
        finally:
            self.field_friend.y_axis.stop()
            self.field_friend.z_axis.stop()

    async def drive_and_punch(self, x: float, y: float, depth: float = None, speed: float = None) -> None:
        await self.drive_to_punch(x)
        await self.punch(y, depth, speed)
        await self.field_friend.y_axis.move_to(self.field_friend.y_axis.MAX_Y, speed)
