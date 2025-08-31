
from __future__ import annotations

from typing import TYPE_CHECKING, Any

import rosys
from nicegui import ui
from rosys.analysis import track
from rosys.geometry import Point

from ...hardware import Axis
from .weeding_implement import ImplementException, WeedingImplement

if TYPE_CHECKING:
    from ...system import System


class WeedingScrew(WeedingImplement):
    DRILL_DEPTH = 0.14
    MAX_CROP_DISTANCE = 0.0

    def __init__(self, system: System) -> None:
        super().__init__('Weed Screw', system)
        self.relevant_weeds = system.plant_locator.weed_category_names
        self.log.debug('Using relevant weeds: %s', self.relevant_weeds)
        self.drill_depth: float = self.DRILL_DEPTH
        self.max_crop_distance: float = self.MAX_CROP_DISTANCE

    async def start_workflow(self) -> None:
        await super().start_workflow()
        try:
            current_pose = self.system.robot_locator.pose
            punch_position = current_pose.transform3d(rosys.geometry.Point3d(x=self.system.field_friend.WORK_X,
                                                                             y=self.next_punch_y_position, z=0))
            self.last_punches.append(punch_position)
            await self.system.puncher.punch(y=self.next_punch_y_position, depth=self.drill_depth)
            self.log.debug(f'removing weeds at screw world position {punch_position} '
                           f'with radius {self.system.field_friend.DRILL_RADIUS}')
            for weed in self.system.plant_provider.get_relevant_weeds(current_pose.point_3d(), min_confidence=0.0):
                if weed.position.distance(punch_position) > self.system.field_friend.DRILL_RADIUS:
                    continue
                self.system.plant_provider.remove_weed(weed.id)
            if isinstance(self.system.detector, rosys.vision.DetectorSimulation):
                self.system.detector.simulated_objects = [
                    obj for obj in self.system.detector.simulated_objects
                    if obj.position.projection().distance(punch_position.projection()) > self.system.field_friend.DRILL_RADIUS]
            # NOTE no weeds to work on at this position -> advance robot
        except Exception as e:
            raise ImplementException(f'Error in Weed Screw Workflow: {e}') from e

    @track
    async def get_target(self) -> Point | None:
        """Return the target position to drive to."""
        self.has_plants_to_handle()
        current_pose = self.system.robot_locator.pose
        weeds_in_range = {weed_id: position for weed_id, position in self.weeds_to_handle.items()
                          if self.system.field_friend.can_reach(position.projection())}
        if not weeds_in_range:
            self.log.debug('No weeds in range')
            return None
        self.log.debug('Found %s weeds in range: %s', len(weeds_in_range),
                       ', '.join(f'{weed_id[:8]}: {(position.x - self.system.field_friend.WORK_X):.6f} m'
                                 for weed_id, position in weeds_in_range.items()))
        for next_weed_id, next_weed_position in weeds_in_range.items():
            weed_world_position = current_pose.transform3d(next_weed_position)
            crops = self.system.plant_provider.get_relevant_crops(current_pose.point_3d())
            if self.cultivated_crop and self.max_crop_distance > 0 \
                    and not any(c.position.distance(weed_world_position) < self.max_crop_distance for c in crops):
                self.log.debug('Skipping weed because it is to far from the cultivated crops')
                continue
            if any(p.distance(weed_world_position) < self.system.field_friend.DRILL_RADIUS for p in self.last_punches):
                self.log.debug('Skipping weed because it was already punched')
                continue
            relative_x = next_weed_position.x - self.system.field_friend.WORK_X
            if relative_x < - self.system.field_friend.DRILL_RADIUS:
                self.log.debug('Skipping weed %s. It is behind the robot: %.6f m', next_weed_id[:8], relative_x)
                continue
            if relative_x < - self.system.driver.parameters.minimum_drive_distance:  # TODO: quickfix for weeds behind the robot
                self.log.debug('Skipping weed %s. It is too far behind the robot: %.6f m', next_weed_id[:8], relative_x)
                continue
            self.log.debug('Targeting weed %s which is %.6f m away at world: %s, local: %s',
                           next_weed_id[:8], relative_x, weed_world_position, next_weed_position)
            self.next_punch_y_position = next_weed_position.y
            return weed_world_position.projection()
        return None

    def settings_ui(self):
        super().settings_ui()
        assert isinstance(self.system.field_friend.z_axis, Axis)
        ui.number('Drill depth', format='%.2f', step=0.01, suffix='m',
                  min=self.system.field_friend.z_axis.max_position,
                  max=self.system.field_friend.z_axis.min_position*-1,
                  on_change=self.request_backup) \
            .props('dense outlined') \
            .classes('w-24') \
            .bind_value(self, 'drill_depth') \
            .bind_visibility_from(self.puncher, 'is_demo', backward=lambda x: not x) \
            .tooltip(f'Set the drill depth for the weeding automation (default: {self.DRILL_DEPTH}m)')
        ui.number('Distance from crop', step=0.005, min=0.0, max=1.00, format='%.3f', on_change=self.request_backup) \
            .props('dense outlined suffix=m') \
            .classes('w-32') \
            .bind_value(self, 'max_crop_distance') \
            .bind_visibility_from(self, 'cultivated_crop') \
            .tooltip('Set the maximum distance a weed can be away from a crop to be considered for weeding. Set to 0 to disable.')

    def backup_to_dict(self) -> dict[str, Any]:
        return super().backup_to_dict() | {
            'drill_depth': self.drill_depth,
            'max_crop_distance': self.max_crop_distance,
        }

    def restore_from_dict(self, data: dict[str, Any]) -> None:
        super().restore_from_dict(data)
        self.drill_depth = data.get('drill_depth', self.drill_depth)
        self.max_crop_distance = data.get('max_crop_distance', self.max_crop_distance)
