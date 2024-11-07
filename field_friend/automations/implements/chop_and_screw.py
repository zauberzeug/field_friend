
from copy import deepcopy
from typing import TYPE_CHECKING, Any

import rosys

from ..puncher import PuncherException
from .weeding_implement import ImplementException, WeedingImplement

if TYPE_CHECKING:
    from ...system import System


class ChopAndScrew(WeedingImplement):

    def __init__(self, system: 'System') -> None:
        super().__init__('Tornado', system)

    async def _perform_workflow(self) -> None:
        self.log.info('Starting dual mechanism workflow...')
        try:
            moved = False
            starting_position = deepcopy(self.system.odometer.prediction)
            if self.crops_to_handle:
                next_crop_position = next(iter(self.crops_to_handle.values()))
                # first: check if weeds near crop
                if self.with_drilling:
                    self.log.info(f'Drilling allowed: only drilling is {self.with_drilling}')
                    self._keep_crops_safe()
                    weeds_in_range = {weed_id: position for weed_id, position in self.weeds_to_handle.items() if next_crop_position.x - self.system.field_friend.DRILL_RADIUS*2
                                      < position.x < next_crop_position.x + self.system.field_friend.DRILL_RADIUS*2 and self.system.field_friend.can_reach(position)}
                    self.log.info(f'weed_position in range: {weeds_in_range.items()}')
                    if weeds_in_range:
                        self.log.info(f' {len(weeds_in_range)} Weeds in range for drilling')
                        while weeds_in_range:
                            next_weed_id, next_weed_position = next(iter(self.crops_to_handle.items()))
                            self.log.info(f'Next weed position: {next_weed_position}')
                            weed_world_position = starting_position.transform(next_weed_position)
                            corrected_relative_weed_position = self.system.odometer.prediction.relative_point(
                                weed_world_position)
                            self.log.info(f'corrected relative weed position: {corrected_relative_weed_position}')
                            moved = True
                            if not self.only_monitoring:
                                await self.system.puncher.drive_and_punch(plant_id=next_weed_id,
                                                                          x=corrected_relative_weed_position.x, y=next_weed_position.y, depth=self.weed_screw_depth, backwards_allowed=False)
                            punched_weeds = [weed_id for weed_id, position in weeds_in_range.items(
                            ) if position.distance(next_weed_position) < self.system.field_friend.DRILL_RADIUS]
                            for weed_id in punched_weeds:
                                self.system.plant_provider.remove_weed(weed_id)
                                if weed_id in weeds_in_range:
                                    del weeds_in_range[weed_id]
                        await self.system.puncher.clear_view()
                # second: check if weed before crop to chop
                if self.with_chopping:
                    self.log.info(f'Chopping allowed: only chopping is {self.with_chopping}')
                    weeds_in_range = {weed_id: position for weed_id, position in self.weeds_to_handle.items() if position.x < next_crop_position.x - (
                        self.system.field_friend.DRILL_RADIUS) and self.system.field_friend.can_reach(position, second_tool=True)}
                    if weeds_in_range:
                        self.log.info('Weeds in range for chopping before crop')
                        crop_world_position = starting_position.transform(next_crop_position)
                        corrected_relative_crop_position = self.system.odometer.prediction.relative_point(
                            crop_world_position)
                        target_position = corrected_relative_crop_position.x - \
                            self.system.field_friend.DRILL_RADIUS - self.system.field_friend.CHOP_RADIUS
                        axis_distance = target_position - self.system.field_friend.WORK_X_CHOP
                        if axis_distance >= 0:
                            local_target = rosys.geometry.Point(x=axis_distance, y=0)
                            world_target = self.system.driver.prediction.transform(local_target)
                            moved = True
                            await self.system.driver.drive_to(world_target)
                            if not self.only_monitoring:
                                await self.system.puncher.chop()
                            choped_weeds = [weed_id for weed_id, position in self.weeds_to_handle.items(
                            ) if target_position - self.system.field_friend.CHOP_RADIUS < self.system.odometer.prediction.relative_point(starting_position.transform(position)).x < target_position + self.system.field_friend.CHOP_RADIUS]
                            for weed_id in choped_weeds:
                                self.system.plant_provider.remove_weed(weed_id)
                        else:
                            self.log.warning(f'Weed position {next_weed_position} is behind field friend')
                    if not moved:
                        await self._follow_line_of_crops()
                        moved = True
            elif self.weeds_to_handle and self.chop_if_no_crops:
                self.log.info('No crops in range, checking for weeds...')
                weeds_in_range = {weed_id: position for weed_id, position in self.weeds_to_handle.items() if self.system.field_friend.WORK_X_CHOP <
                                  position.x < self.system.field_friend.WORK_X + self.WORKING_DISTANCE and self.system.field_friend.can_reach(position)}
                if weeds_in_range:
                    next_weed_position = next(iter(weeds_in_range.values()))
                    axis_distance = next_weed_position.x - self.system.field_friend.WORK_X_CHOP + self.system.field_friend.CHOP_RADIUS
                    if axis_distance >= 0:
                        local_target = rosys.geometry.Point(x=axis_distance, y=0)
                        self.log.info(f'Next weed position: {next_weed_position}')
                        self.log.info(f'Axis distance: {axis_distance}')
                        world_target = self.system.driver.prediction.transform(local_target)
                        moved = True
                        await self.system.driver.drive_to(world_target)
                        if not self.only_monitoring:
                            await self.system.puncher.chop()
                        choped_weeds = [weed_id for weed_id, position in self.weeds_to_handle.items(
                        ) if axis_distance - self.system.field_friend.CHOP_RADIUS < self.system.odometer.prediction.relative_point(starting_position.transform(position)).x < axis_distance + self.system.field_friend.CHOP_RADIUS]
                        for weed_id in choped_weeds:
                            self.system.plant_provider.remove_weed(weed_id)
                    else:
                        self.log.warning(f'Weed position {next_weed_position} is behind field friend')
            if not moved:
                await self._driving_a_bit_forward()
            await rosys.sleep(0.2)
            self.log.info('Workflow completed')
        except PuncherException as e:
            self.log.error(f'Error while Dual Mechanism Workflow: {e}')
        except Exception as e:
            raise ImplementException(f'Error while double mechanism Workflow: {e}') from e

    def backup(self) -> dict:
        return super().backup() | {
            'drill_with_open_tornado': self.drill_with_open_tornado,
            'drill_between_crops': self.drill_between_crops,
        }

    def restore(self, data: dict[str, Any]) -> None:
        super().restore(data)
        self.drill_with_open_tornado = data.get('drill_with_open_tornado', self.drill_with_open_tornado)
        self.drill_between_crops = data.get('drill_between_crops', self.drill_between_crops)

    def _has_plants_to_handle(self) -> bool:
        super()._has_plants_to_handle()
        return any(self.crops_to_handle)
