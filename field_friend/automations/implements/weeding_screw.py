
from __future__ import annotations

from typing import TYPE_CHECKING, Any

import rosys
from nicegui import ui

from ...hardware import Tornado
from .weeding_implement import ImplementException, WeedingImplement

if TYPE_CHECKING:
    from ...system import System


class WeedingScrew(WeedingImplement):

    def __init__(self, system: System) -> None:
        super().__init__('Weed Screw', system, 'weeding_screw')
        self.relevant_weeds = system.plant_locator.weed_category_names
        self.log.info(f'Using relevant weeds: {self.relevant_weeds}')
        self.weed_screw_depth: float = 0.13
        self.max_crop_distance: float = 0.08

    async def start_workflow(self) -> None:
        await super().start_workflow()
        try:
            punch_position = self.system.odometer.prediction.transform3d(
                rosys.geometry.Point3d(x=self.system.field_friend.WORK_X, y=self.next_punch_y_position, z=0))
            self.last_punches.append(punch_position)
            await self.system.puncher.punch(y=self.next_punch_y_position, depth=self.weed_screw_depth)
            punched_weeds = [weed.id for weed in self.system.plant_provider.get_relevant_weeds(self.system.odometer.prediction.point_3d())
                             if weed.position.distance(punch_position) <= self.system.field_friend.DRILL_RADIUS]
            for weed_id in punched_weeds:
                self.system.plant_provider.remove_weed(weed_id)
            if isinstance(self.system.detector, rosys.vision.DetectorSimulation):
                self.log.info(f'removing weeds at screw world position {punch_position} '
                              f'with radius {self.system.field_friend.DRILL_RADIUS}')
                self.system.detector.simulated_objects = [
                    obj for obj in self.system.detector.simulated_objects
                    if obj.position.projection().distance(punch_position.projection()) > self.system.field_friend.DRILL_RADIUS]
            # NOTE no weeds to work on at this position -> advance robot
        except Exception as e:
            raise ImplementException(f'Error in Weed Screw Workflow: {e}') from e

    async def get_stretch(self, max_distance: float) -> float:
        await super().get_stretch(max_distance)
        super()._has_plants_to_handle()
        weeds_in_range = {weed_id: position for weed_id, position in self.weeds_to_handle.items()
                          if self.system.field_friend.can_reach(position.projection())}
        if not weeds_in_range:
            self.log.info('No weeds in range')
            return self.WORKING_DISTANCE
        self.log.info(f'Found {len(weeds_in_range)} weeds in range: {weeds_in_range}')
        for next_weed_id, next_weed_position in weeds_in_range.items():
            # next_weed_position.x += 0.01  # NOTE somehow this helps to mitigate an offset we experienced in the tests
            weed_world_position = self.system.odometer.prediction.transform3d(next_weed_position)
            crops = self.system.plant_provider.get_relevant_crops(self.system.odometer.prediction.point_3d())
            if self.cultivated_crop and not any(c.position.distance(weed_world_position) < self.max_crop_distance for c in crops):
                self.log.info('Skipping weed because it is to far from the cultivated crops')
                continue
            if any(p.distance(weed_world_position) < self.system.field_friend.DRILL_RADIUS for p in self.last_punches):
                self.log.info('Skipping weed because it was already punched')
                continue
            stretch = next_weed_position.x - self.system.field_friend.WORK_X
            if stretch < - self.system.field_friend.DRILL_RADIUS:
                self.log.info(f'Skipping weed {next_weed_id} because it is behind the robot')
                continue
            stretch = max(stretch, 0)
            self.log.info(f'Targeting weed {next_weed_id} which is {stretch} away at world: '
                          f'{weed_world_position}, local: {next_weed_position}')
            if stretch < max_distance:
                self.next_punch_y_position = next_weed_position.y
                return stretch
            break
        return self.WORKING_DISTANCE

    def settings_ui(self):
        super().settings_ui()
        # TODO: handle Tornado case -> no max_position property
        if self.system.field_friend.z_axis and not isinstance(self.system.field_friend.z_axis, Tornado):
            ui.number('Drill depth', format='%.2f', step=0.01,
                      min=self.system.field_friend.z_axis.max_position,
                      max=self.system.field_friend.z_axis.min_position*-1,
                      on_change=self.request_backup) \
                .props('dense outlined suffix=°') \
                .classes('w-24') \
                .bind_value(self, 'weed_screw_depth') \
                .tooltip('Set the drill depth for the weeding automation')
        ui.number('Maximum weed distance from crop', step=0.001, min=0.001, max=1.00, format='%.3f', on_change=self.request_backup) \
            .props('dense outlined suffix=m') \
            .classes('w-24') \
            .bind_value(self, 'max_crop_distance') \
            .tooltip('Set the maximum distance a weed can be away from a crop to be considered for weeding')

    def backup(self) -> dict:
        return super().backup() | {
            'weed_screw_depth': self.weed_screw_depth,
            'max_crop_distance': self.max_crop_distance,
        }

    def restore(self, data: dict[str, Any]) -> None:
        super().restore(data)
        self.weed_screw_depth = data.get('weed_screw_depth', self.weed_screw_depth)
        self.max_crop_distance = data.get('max_crop_distance', self.max_crop_distance)
