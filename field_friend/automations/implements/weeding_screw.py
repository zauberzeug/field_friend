
from collections import deque
from typing import TYPE_CHECKING, Any

import rosys
from nicegui import ui

from . import ImplementException, WeedingImplement

if TYPE_CHECKING:
    from system import System


class WeedingScrew(WeedingImplement):

    def __init__(self, system: 'System') -> None:
        super().__init__('Weed Screw', system, 'weeding_screw')
        self.relevant_weeds = system.small_weed_category_names + system.big_weed_category_names
        self.log.info(f'Using relevant weeds: {self.relevant_weeds}')
        self.weed_screw_depth: float = 0.13
        self.crop_safety_distance: float = 0.01
        self.max_crop_distance: float = 0.08
        self.last_punches: deque[rosys.geometry.Point] = deque(maxlen=5)
        self.next_weed_id: str = ''

    async def start_workflow(self) -> bool:
        await super().start_workflow()
        try:
            if self.next_weed_id not in self.weeds_to_handle:
                self.log.error('next weed not found in weeds_to_handle')
                return True
            next_weed_position = self.weeds_to_handle[self.next_weed_id]
            weed_world_position = self.system.odometer.prediction.transform(next_weed_position)
            self.last_punches.append(weed_world_position)
            await self.system.puncher.punch(y=next_weed_position.y, depth=self.weed_screw_depth, plant_id=self.next_weed_id)
            # TODO: check if weeds_in_range is needed
            weeds_in_range = {weed_id: position for weed_id, position in self.weeds_to_handle.items()
                            if self.system.field_friend.can_reach(position)}
            punched_weeds = [weed_id for weed_id, position in weeds_in_range.items()
                                if position.distance(next_weed_position) <= self.system.field_friend.DRILL_RADIUS
                                or weed_id == self.next_weed_id]
            for weed_id in punched_weeds:
                self.system.plant_provider.remove_weed(weed_id)
                if weed_id in weeds_in_range:
                    del weeds_in_range[weed_id]
                self.kpi_provider.increment_weeding_kpi('weeds_removed')
            if isinstance(self.system.detector, rosys.vision.DetectorSimulation):
                screw_world_position = self.system.odometer.prediction.transform(
                    rosys.geometry.Point(x=self.system.field_friend.WORK_X, y=self.system.field_friend.y_axis.position))
                self.log.info(f'removing weeds at screw world position {screw_world_position} '
                                f'with radius {self.system.field_friend.DRILL_RADIUS}')
                self.system.detector.simulated_objects = [
                    obj for obj in self.system.detector.simulated_objects
                    if obj.position.projection().distance(screw_world_position) > self.system.field_friend.DRILL_RADIUS]
            return True  # NOTE no weeds to work on at this position -> advance robot
        except Exception as e:
            raise ImplementException(f'Error while Weed Screw Workflow: {e}') from e
        
    def get_stretch(self) -> float:
        super()._has_plants_to_handle()
        weeds_in_range = {weed_id: position for weed_id, position in self.weeds_to_handle.items()
                            if self.system.field_friend.can_reach(position)}
        if not weeds_in_range:
            self.log.info('No weeds in range')
            return self.WORKING_DISTANCE
        
        for next_weed_id, next_weed_position in weeds_in_range.items():
            # next_weed_position.x += 0.01  # NOTE somehow this helps to mitigate an offset we experienced in the tests
            weed_world_position = self.system.odometer.prediction.transform(next_weed_position)
            crops = self.system.plant_provider.get_relevant_crops(self.system.odometer.prediction.point)
            if self.cultivated_crop and not any(c.position.distance(weed_world_position) < self.max_crop_distance for c in crops):
                self.log.info('Skipping weed because it is to far from the cultivated crops')
                continue
            if any(p.distance(weed_world_position) < self.crop_safety_distance for p in self.last_punches):
                self.log.info('Skipping weed because it was already punched')
                continue
            self.log.info(f'Targeting weed {next_weed_id} at world: {weed_world_position}, local: {next_weed_position}')
            self.next_weed_id = next_weed_id
            return next_weed_position.x - self.system.field_friend.WORK_X
        return self.WORKING_DISTANCE

    def settings_ui(self):
        super().settings_ui()
        ui.number('Drill depth', format='%.2f', step=0.01,
                  min=self.system.field_friend.z_axis.max_position,
                  max=self.system.field_friend.z_axis.min_position*-1,
                  on_change=self.request_backup) \
            .props('dense outlined suffix=°') \
            .classes('w-24') \
            .bind_value(self, 'weed_screw_depth') \
            .tooltip('Set the drill depth for the weeding automation')
        ui.number('Crop safety distance', step=0.001, min=0.001, max=0.05, format='%.3f', on_change=self.request_backup) \
            .props('dense outlined suffix=m') \
            .classes('w-24') \
            .bind_value(self, 'crop_safety_distance') \
            .tooltip('Set the crop safety distance for the weeding automation')
        ui.number('Maximum weed distance from crop', step=0.001, min=0.001, max=1.00, format='%.3f', on_change=self.request_backup) \
            .props('dense outlined suffix=m') \
            .classes('w-24') \
            .bind_value(self, 'max_crop_distance') \
            .tooltip('Set the maximum distance a weed can be away from a crop to be considered for weeding')

    def backup(self) -> dict:
        return super().backup() | {
            'weed_screw_depth': self.weed_screw_depth,
            'crop_safety_distance': self.crop_safety_distance,
            'max_crop_distance': self.max_crop_distance,
        }

    def restore(self, data: dict[str, Any]) -> None:
        super().restore(data)
        self.weed_screw_depth = data.get('weed_screw_depth', self.weed_screw_depth)
        self.crop_safety_distance = data.get('crop_safety_distance', self.crop_safety_distance)
        self.max_crop_distance = data.get('max_crop_distance', self.max_crop_distance)
