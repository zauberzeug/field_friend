import logging

import rosys
from rosys.driving import Driver
from rosys.geometry import Point

from ..hardware import ChainAxis, FieldFriend, Tornado, YAxis, ZAxis
from .kpi_provider import KpiProvider


class PuncherException(Exception):
    pass


class Puncher:
    def __init__(self, field_friend: FieldFriend, driver: Driver, kpi_provider: KpiProvider) -> None:
        self.field_friend = field_friend
        self.driver = driver
        self.kpi_provider = kpi_provider
        self.log = logging.getLogger('field_friend.puncher')

    async def try_home(self) -> bool:
        if self.field_friend.y_axis is None or self.field_friend.z_axis is None:
            rosys.notify('no y or z axis', 'negative')
            return False
        try:
            if self.field_friend.estop.active:
                rosys.notify('Estop active, relaese first', 'negative')
                return False
            if not await self.field_friend.z_axis.try_reference():
                return False
            await rosys.sleep(0.2)
            if not await self.field_friend.y_axis.try_reference():
                return False
            return True
        except Exception as e:
            raise PuncherException('homing failed') from e
        finally:
            await self.field_friend.y_axis.stop()
            await self.field_friend.z_axis.stop()

    async def drive_to_punch(self, local_target_x: float) -> None:
        if self.field_friend.y_axis is None or self.field_friend.z_axis is None:
            rosys.notify('no y or z axis', 'negative')
            return
        self.log.info(f'Driving to punch at {local_target_x}...')
        work_x = self.field_friend.WORK_X
        if local_target_x < work_x:
            self.log.info(f'Target: {local_target_x} is behind')
        axis_distance = local_target_x - work_x
        local_target = Point(x=axis_distance, y=0)
        world_target = self.driver.prediction.transform(local_target)
        await self.driver.drive_to(world_target, backward=axis_distance < 0)

    async def punch(self, y: float, *, depth: float = 0.01, angle: float = 180, turns: float = 2.0) -> None:
        self.log.info(f'Punching at {y} with depth {depth}...')
        if self.field_friend.y_axis is None or self.field_friend.z_axis is None:
            rosys.notify('no y or z axis', 'negative')
            self.log.warning('no y or z axis')
            return
        try:
            if not self.field_friend.y_axis.is_referenced or not self.field_friend.z_axis.is_referenced:
                rosys.notify('axis are not referenced, homing!', type='info')
                self.log.info('axis are not referenced, homing!')
                success = await self.try_home()
                if not success:
                    rosys.notify('homing failed!', type='negative')
                    self.log.error('homing failed!')
                    raise PuncherException('homing failed')
                await rosys.sleep(0.5)
            if isinstance(self.field_friend.y_axis, ChainAxis):
                if not self.field_friend.y_axis.min_position <= y <= self.field_friend.y_axis.max_position:
                    rosys.notify('y position out of range', type='negative')
                    raise PuncherException('y position out of range')
            elif isinstance(self.field_friend.y_axis, YAxis):
                if not self.field_friend.y_axis.min_position <= y <= self.field_friend.y_axis.max_position:
                    rosys.notify('y position out of range', type='negative')
                    raise PuncherException('y position out of range')

            if isinstance(self.field_friend.z_axis, Tornado):
                await self.field_friend.y_axis.move_to(y)
                await self.tornado_drill(angle=angle, turns=turns)
            elif isinstance(self.field_friend.z_axis, ZAxis):
                await self.field_friend.y_axis.move_to(y)
                await self.field_friend.z_axis.move_to(-depth)
                await self.field_friend.z_axis.return_to_reference()
            self.log.info(f'punched successfully at {y:.2f} with depth {depth}')
            self.kpi_provider.increment_weeding_kpi('punches')
        except Exception as e:
            raise PuncherException(f'punching failed because: {e}') from e
        finally:
            await self.field_friend.y_axis.stop()
            await self.field_friend.z_axis.stop()

    async def clear_view(self) -> None:
        if self.field_friend.y_axis is None:
            rosys.notify('no y axis', 'negative')
            return
        self.log.info('Clearing view...')
        if isinstance(self.field_friend.y_axis, ChainAxis):
            await self.field_friend.y_axis.return_to_reference()
            return
        elif isinstance(self.field_friend.y_axis, YAxis):
            y = self.field_friend.y_axis.min_position if self.field_friend.y_axis.position <= 0 else self.field_friend.y_axis.max_position
            await self.field_friend.y_axis.move_to(y, speed=self.field_friend.y_axis.max_speed)
        await self.field_friend.y_axis.stop()

    async def drive_and_punch(self, x: float, y: float, depth: float = 0.05, angle: float = 180, turns: float = 2.0, backwards_allowed: bool = True) -> None:
        if self.field_friend.y_axis is None or self.field_friend.z_axis is None:
            rosys.notify('no y or z axis', 'negative')
            return
        try:
            work_x = self.field_friend.WORK_X
            if x < work_x and not backwards_allowed:
                self.log.warning(f'target x: {x} is behind')
                return
            await self.drive_to_punch(x)
            await self.punch(y, depth=depth, angle=angle, turns=turns)
            # await self.clear_view()
        except Exception as e:
            raise PuncherException('drive and punch failed') from e

    async def chop(self) -> None:
        if not isinstance(self.field_friend.y_axis, ChainAxis):
            raise PuncherException('chop is only available for chain axis')
        if self.field_friend.y_axis.position <= 0:
            await self.field_friend.y_axis.move_dw_to_l_ref()
        else:
            await self.field_friend.y_axis.move_dw_to_r_ref()
        await self.field_friend.y_axis.stop()
        self.kpi_provider.increment_weeding_kpi('chops')

    async def tornado_drill(self, angle: float = 180, turns: float = 2) -> None:
        self.log.info(f'Drilling with tornado at {angle}...')
        if not isinstance(self.field_friend.z_axis, Tornado):
            raise PuncherException('tornado drill is only available for tornado axis')
        try:
            if not self.field_friend.z_axis.is_referenced:
                rosys.notify('axis are not referenced, homing!', type='info')
                success = await self.try_home()
                if not success:
                    rosys.notify('homing failed!', type='negative')
                    raise PuncherException('homing failed')
                await rosys.sleep(0.5)
            await self.field_friend.z_axis.move_down_until_reference()

            await self.field_friend.z_axis.turn_knifes_to(angle)
            await rosys.sleep(3)
            await self.field_friend.z_axis.turn_by(turns)
            await rosys.sleep(3)

            await self.field_friend.z_axis.return_to_reference()
            await rosys.sleep(0.5)
            if not await self.field_friend.z_axis.try_reference_turn():
                raise PuncherException('tornado reference failed')
        except Exception as e:
            raise PuncherException(f'tornado drill failed because of: {e}') from e
        finally:
            await self.field_friend.y_axis.stop()
            await self.field_friend.z_axis.stop()
