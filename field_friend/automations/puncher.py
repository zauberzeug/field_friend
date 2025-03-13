import logging
import os

import rosys
from rosys.analysis import track
from rosys.driving import Driver
from rosys.geometry import Point

from ..hardware import Axis, ChainAxis, FieldFriend, Tornado


class PuncherException(Exception):
    pass


class Puncher:
    def __init__(self, field_friend: FieldFriend, driver: Driver) -> None:
        self.punch_allowed: str = 'waiting'
        self.field_friend = field_friend
        self.driver = driver
        self.log = logging.getLogger('field_friend.puncher')
        self.is_demo = False

    @track
    async def try_home(self) -> bool:
        if self.field_friend.y_axis is None or self.field_friend.z_axis is None:
            rosys.notify('no y or z axis', 'negative')
            return False
        try:
            if self.field_friend.estop.active:
                rosys.notify('Estop active, release first', 'negative')
                return False
            if not await self.field_friend.z_axis.try_reference():
                return False
            await rosys.sleep(0.5)
            if not await self.field_friend.y_axis.try_reference():
                return False
            return True
        except Exception as e:
            rosys.notify('Homing failed', 'negative')
            raise PuncherException('homing failed') from e
        finally:
            await self.field_friend.y_axis.stop()
            await self.field_friend.z_axis.stop()

    @track
    async def drive_to_punch(self, local_target_x: float) -> None:
        if self.field_friend.y_axis is None or self.field_friend.z_axis is None:
            rosys.notify('no y or z axis', 'negative')
            return
        self.log.info(f'Driving to punch at {local_target_x:.2f}...')
        work_x = self.field_friend.WORK_X
        if local_target_x < work_x:
            self.log.info(f'Target: {local_target_x} is behind')
        axis_distance = local_target_x - work_x
        local_target = Point(x=axis_distance, y=0)
        world_target = self.driver.prediction.transform(local_target)
        with self.driver.parameters.set(linear_speed_limit=0.125, angular_speed_limit=0.1):
            await self.driver.drive_to(world_target, backward=axis_distance < 0)

    @track
    async def punch(self,
                    y: float, *,
                    depth: float = 0.01,
                    angle: float = 180,
                    turns: float = 2.0,
                    with_open_tornado: bool = False,
                    ) -> None:
        y += self.field_friend.WORK_Y
        y = round(y, 5)
        self.log.debug(f'Punching at {y} with depth {depth}...')
        rest_position = 'reference'
        if self.field_friend.y_axis is None or self.field_friend.z_axis is None:
            rosys.notify('no y or z axis', 'negative')
            self.log.warning('no y or z axis')
            return
        try:
            if not self.field_friend.y_axis.is_referenced or not self.field_friend.z_axis.is_referenced:
                rosys.notify('axis are not referenced, homing!', type='info')
                success = await self.try_home()
                if not success:
                    raise PuncherException('homing failed')
                # await rosys.sleep(0.5)
            if isinstance(self.field_friend.y_axis, ChainAxis):
                if not self.field_friend.y_axis.min_position <= y <= self.field_friend.y_axis.max_position:
                    rosys.notify('y position out of range', type='negative')
                    raise PuncherException('y position out of range')
            elif isinstance(self.field_friend.y_axis, Axis):
                if not self.field_friend.y_axis.min_position <= y <= self.field_friend.y_axis.max_position:
                    rosys.notify('y position out of range', type='negative')
                    raise PuncherException('y position out of range')

            if isinstance(self.field_friend.z_axis, Tornado):
                await self.field_friend.y_axis.move_to(y)
                await self.tornado_drill(angle=angle, turns=turns, with_open_drill=with_open_tornado)

            elif isinstance(self.field_friend.z_axis, Axis):
                if self.is_demo:
                    self.log.warning('punching with demo mode is not yet implemented for z axis')
                await self.field_friend.y_axis.move_to(y)
                await self.field_friend.z_axis.move_to(-depth)
                if os.environ.get('Z_AXIS_REST_POSITION'):
                    target = float(os.environ.get('Z_AXIS_REST_POSITION', '0'))
                    await self.field_friend.z_axis.move_to(target)
                    rest_position = f'custom position {target}'
                else:
                    await self.field_friend.z_axis.return_to_reference()
            self.log.debug(f'punched at {y:.2f} with depth {depth}, now back to rest position "{rest_position}"')
        except Exception as e:
            raise PuncherException('punching failed') from e
        finally:
            await self.field_friend.y_axis.stop()
            await self.field_friend.z_axis.stop()

    @track
    async def clear_view(self) -> None:
        if self.field_friend.y_axis is None:
            rosys.notify('no y axis', 'negative')
            return
        self.log.debug('Clearing view...')
        if isinstance(self.field_friend.y_axis, ChainAxis):
            await self.field_friend.y_axis.return_to_reference()
            return
        if isinstance(self.field_friend.y_axis, Axis):
            if isinstance(self.field_friend.z_axis, Axis):
                if self.field_friend.z_axis.position != 0:
                    await self.field_friend.z_axis.return_to_reference()
            y = self.field_friend.y_axis.min_position if self.field_friend.y_axis.position <= 0 else self.field_friend.y_axis.max_position
            await self.field_friend.y_axis.move_to(y, speed=self.field_friend.y_axis.max_speed)
        await self.field_friend.y_axis.stop()

    @track
    async def chop(self) -> None:
        if not isinstance(self.field_friend.y_axis, ChainAxis):
            raise PuncherException('chop is only available for chain axis')
        if self.field_friend.y_axis.position <= 0:
            await self.field_friend.y_axis.move_dw_to_l_ref()
        else:
            await self.field_friend.y_axis.move_dw_to_r_ref()
        await self.field_friend.y_axis.stop()

    @track
    async def tornado_drill(self, angle: float = 180, turns: float = 2, with_open_drill=False) -> None:
        self.log.debug(f'Drilling with tornado at {angle}°...')
        if not isinstance(self.field_friend.z_axis, Tornado):
            raise PuncherException('tornado drill is only available for tornado axis')
        try:
            if not self.field_friend.z_axis.is_referenced:
                rosys.notify('axis are not referenced, homing!', type='info')
                success = await self.try_home()
                if not success:
                    raise PuncherException('homing failed')
                await rosys.sleep(0.5)
            await self.field_friend.z_axis.move_down_until_reference(min_position=-0.058 if self.is_demo else None)

            await self.field_friend.z_axis.turn_knifes_to(angle)
            await rosys.sleep(2)
            await self.field_friend.z_axis.turn_by(turns)
            await rosys.sleep(2)

            if with_open_drill:
                self.log.debug('Drilling again with open drill...')
                await self.field_friend.z_axis.turn_knifes_to(0)
                await rosys.sleep(2)
                await self.field_friend.z_axis.turn_by(turns)
                await rosys.sleep(2)

            await self.field_friend.z_axis.return_to_reference()
            await rosys.sleep(0.5)
            await self.field_friend.z_axis.turn_knifes_to(0)
            await rosys.sleep(0.5)
        except Exception as e:
            raise PuncherException(f'tornado drill failed because of: {e}') from e
        finally:
            if self.field_friend.y_axis:
                await self.field_friend.y_axis.stop()
            await self.field_friend.z_axis.stop()
