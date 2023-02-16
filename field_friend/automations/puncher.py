import rosys
from rosys.driving import Driver
from rosys.geometry import Point
from rosys.hardware import Wheels

from ..hardware import EStop, YAxis, ZAxis


class Puncher:
    def __init__(self, y_axis: YAxis, z_axis: ZAxis, e_stop: EStop, driver: Driver) -> None:
        self.y_axis = y_axis
        self.z_axis = z_axis
        self.driver = driver

    async def home(self) -> None:
        try:
            if not await self.y_axis.try_reference():
                return False
            if not await self.z_axis.try_reference():
                return False
        except Exception as e:
            raise Exception('homing failed') from e
        finally:
            self.y_axis.stop()
            self.z_axis.stop()

    async def drive_to_punch(self, target_distance_x: float) -> None:
        min_drive_distance = 0.1
        axis_distance = max(target_distance_x - self.y_axis.AXIS_OFFSET_X, min_drive_distance)
        target_point = Point(x=axis_distance, y=0)
        await self.driver.drive_to(target_point)
        await rosys.sleep(0.2)
        await self.driver.wheels.stop()

    async def punch(self, y: float, depth: float = None, speed: float = None) -> None:
        try:
            if not self.y_axis.yaxis_is_referenced or not self.z_axis.zaxis_is_referenced:
                rosys.notify('axis are not referenced')
                return
            if speed == None:
                speed = self.z_axis.Z_AXIS_MAX_SPEED
                speed = self.y_axis.Y_AXIS_MAX_SPEED
            await self.y_axis.move_to(y, speed)
            if depth == None:
                depth = self.z_axis.zaxis_drill_depth
            await self.z_axis.move_to(depth, speed)
            await self.z_axis.move_to(self.z_axis.MAX_Z, speed)
        except Exception as e:
            raise Exception('punching failed') from e
        finally:
            self.y_axis.stop()
            self.z_axis.stop()

    async def drive_and_punch(self, x: float, y: float, depth: float = None, speed: float = None) -> None:
        await self.drive_to_punch(x)
        await self.punch(y, depth, speed)
        await self.y_axis.move_to(self.y_axis.MAX_Y, speed)
