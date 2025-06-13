from __future__ import annotations

from typing import TYPE_CHECKING, Any

import rosys
from nicegui import ui
from rosys.analysis import track
from rosys.geometry import Point, Point3d

from .weeding_implement import ImplementException, WeedingImplement

if TYPE_CHECKING:
    from ...system import System


class Tornado(WeedingImplement):
    DRILL_BETWEEN_CROPS = False
    DRILL_DEPTH = 0.0
    DRILL_WITH_OPEN_TORNADO = False
    SKIP_IF_NO_WEEDS = False
    TORNADO_ANGLE = 180.0

    def __init__(self, system: System) -> None:
        super().__init__('Tornado', system)
        self.field_friend = system.field_friend
        self.drill_between_crops: bool = self.DRILL_BETWEEN_CROPS
        self.drill_depth: float = self.DRILL_DEPTH
        self.drill_with_open_tornado: bool = self.DRILL_WITH_OPEN_TORNADO
        self.skip_if_no_weeds: bool = self.SKIP_IF_NO_WEEDS
        self.tornado_angle: float = self.TORNADO_ANGLE

    async def start_workflow(self) -> None:
        await super().start_workflow()
        try:
            # TODO: do we need to set self.next_crop_id = '' on every return?
            punch_position = self.system.robot_locator.pose.transform(
                rosys.geometry.Point(x=self.system.field_friend.WORK_X, y=self.next_punch_y_position))
            self.last_punches.append(Point3d.from_point(punch_position))
            self.log.debug('Drilling crop at %s with angle %.1f°', punch_position, self.tornado_angle)
            await self.system.puncher.punch(y=self.next_punch_y_position, depth=self.drill_depth, angle=self.tornado_angle, with_open_tornado=self.drill_with_open_tornado)
            # TODO remove weeds from plant_provider
            if isinstance(self.system.detector, rosys.vision.DetectorSimulation):
                inner_diameter, outer_diameter = self.field_friend.tornado_diameters(self.tornado_angle)
                if self.drill_with_open_tornado:
                    outer_diameter = self.field_friend.tornado_diameters(0)[1]
                inner_radius = inner_diameter / 2
                outer_radius = outer_diameter / 2
                self.system.detector.simulated_objects = [obj for obj in self.system.detector.simulated_objects
                                                          if not inner_radius <= obj.position.projection().distance(punch_position) <= outer_radius]
        except Exception as e:
            raise ImplementException('Error while tornado Workflow') from e

    def _has_plants_to_handle(self) -> bool:
        super()._has_plants_to_handle()
        if len(self.crops_to_handle) == 0:
            return False
        return True

    @track
    async def get_move_target(self) -> Point | None: # pylint: disable=too-many-return-statements
        """Return the target position to drive to."""
        self._has_plants_to_handle()
        if len(self.crops_to_handle) == 0:
            self.log.debug('No crops to handle')
            return None
        closest_crop_id, closest_crop_position = next(iter(self.crops_to_handle.items()))
        closest_crop_world_position = self.system.robot_locator.pose.transform3d(closest_crop_position)
        if any(p.distance(closest_crop_world_position) < self.field_friend.DRILL_RADIUS for p in self.last_punches):
            self.log.debug('Skipping crop because it was already punched')
            return None
        if not self.system.field_friend.can_reach(closest_crop_position.projection()):
            self.log.debug('Target crop is not in the working area')
            return None
        if self._crops_in_drill_range(closest_crop_id, closest_crop_position.projection(), self.tornado_angle):
            self.log.debug('Crops in punch range')
            return None

        angle = 0 if self.drill_with_open_tornado else self.tornado_angle
        tornado_outer_diameter = self.field_friend.tornado_diameters(angle)[1]
        if self.skip_if_no_weeds and not any(closest_crop_position.distance(weed_position) < tornado_outer_diameter
                                             for weed_position in self.weeds_to_handle.values()):
            self.log.debug('Skipping crop because there are no weeds next to it.')
            return None

        relative_x = closest_crop_position.x - self.system.field_friend.WORK_X
        if relative_x < - self.system.field_friend.DRILL_RADIUS:
            self.log.debug('Skipping crop %s because it is behind the robot', closest_crop_id)
            return None
        self.log.debug('Targeting crop %s which is %.2f away at world: %s, local: %s',
                      closest_crop_id, relative_x, closest_crop_world_position, closest_crop_position)
        self.next_punch_y_position = closest_crop_position.y
        return closest_crop_world_position.projection()

    def _crops_in_drill_range(self, crop_id: str, crop_position: rosys.geometry.Point, angle: float) -> bool:
        inner_diameter, outer_diameter = self.system.field_friend.tornado_diameters(angle)
        crop_world_position = self.system.robot_locator.pose.transform(crop_position)
        for crop in self.system.plant_provider.crops:
            if crop.id != crop_id:
                distance = crop_world_position.distance(crop.position.projection())
                if inner_diameter/2 <= distance <= outer_diameter/2:
                    return True
        return False

    def backup_to_dict(self) -> dict[str, Any]:
        return super().backup_to_dict() | {
            'drill_between_crops': self.drill_between_crops,
            'drill_depth': self.drill_depth,
            'drill_with_open_tornado': self.drill_with_open_tornado,
            'skip_if_no_weeds': self.skip_if_no_weeds,
            'tornado_angle': self.tornado_angle,
        }

    def restore_from_dict(self, data: dict[str, Any]) -> None:
        super().restore_from_dict(data)
        self.drill_between_crops = data.get('drill_between_crops', self.DRILL_BETWEEN_CROPS)
        self.drill_depth = data.get('drill_depth', self.DRILL_DEPTH)
        self.drill_with_open_tornado = data.get('drill_with_open_tornado', self.DRILL_WITH_OPEN_TORNADO)
        self.skip_if_no_weeds = data.get('skip_if_no_weeds', self.SKIP_IF_NO_WEEDS)
        self.tornado_angle = data.get('tornado_angle', self.TORNADO_ANGLE)

    def settings_ui(self):
        super().settings_ui()
        with ui.card().props('flat bordered').classes('w-full'):
            ui.label('Tornado Angle')
            with ui.grid(columns=2).classes('w-full gap-0'):
                ui.label('Knife angle:')
                angle_label = ui.label()
                ui.label('Inner diameter:')
                inner_label = ui.label()
                ui.label('Outer diameter:')
                outer_label = ui.label()

            def get_angle_label() -> None:
                angle = self.tornado_angle
                inner_diameter, outer_diameter = self.field_friend.tornado_diameters(angle)
                if self.drill_with_open_tornado:
                    outer_diameter = self.field_friend.tornado_diameters(0)[1]
                angle_label.set_text(f'{angle}°')
                inner_label.set_text(f'{inner_diameter*100:.1f} cm')
                outer_label.set_text(f'{outer_diameter*100:.1f} cm')

            ui.slider(min=0, max=180, step=1, on_change=get_angle_label) \
                .bind_value(self, 'tornado_angle', forward=lambda v: 180-v, backward=lambda v: abs(v-180)) \
                .tooltip(f'Set the angle for the tornado drill. Moving to the left will decrease the diameter and moving to the right will increase it. (default: {self.TORNADO_ANGLE}°)')
            ui.checkbox('Second drill with the biggest diameter', on_change=get_angle_label) \
                .bind_value(self, 'drill_with_open_tornado') \
                .tooltip(f'Drill a second time with 0° angle (default: {self.DRILL_WITH_OPEN_TORNADO})')

        ui.number('Drill depth', format='%.2f', step=0.01, on_change=self.request_backup) \
            .props('dense outlined suffix=m') \
            .classes('w-24') \
            .bind_value(self, 'drill_depth') \
            .tooltip(f'Set the depth for the tornado drill. 0 is at ground level and positive values are below ground level (default: {self.DRILL_DEPTH:.2f}m)')
        ui.checkbox('Skip if no weeds') \
            .bind_value(self, 'skip_if_no_weeds') \
            .tooltip(f'Skip crops with no weeds in their vicinity (default: {self.SKIP_IF_NO_WEEDS})')
        # ui.checkbox('Drill between crops') \
        #     .bind_value(self, 'drill_between_crops') \
        #     .tooltip('Set the weeding automation to drill between crops')
