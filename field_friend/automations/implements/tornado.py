
import asyncio

from collections import deque
from typing import TYPE_CHECKING, Any

import rosys
from nicegui import ui

from ..puncher import PuncherException
from .weeding_implement import ImplementException, WeedingImplement

if TYPE_CHECKING:
    from system import System


class Tornado(WeedingImplement):

    def __init__(self, system: 'System') -> None:
        super().__init__('Tornado', system)
        self.tornado_angle: float = 30.0
        self.drill_with_open_tornado: bool = False
        self.drill_between_crops: bool = False
        self.field_friend = system.field_friend

        self.move_y_while_driving = False
        self.move_knifes_while_driving = False

        self.y_axis_move_task = None
        self.knife_reset_task = None

    async def _reset_knifes(self) -> None:
        await self.field_friend.z_axis.turn_knifes_to(0)
        await rosys.sleep(0.5)
        await self.field_friend.z_axis.turn_knifes_to(self.tornado_angle)

    async def finish(self) -> None:
        await super().finish()
        if self.y_axis_move_task:
            try:
                self.y_axis_move_task.cancel()
                self.y_axis_move_task = None
            except:
                pass
        if self.knife_reset_task:
            try:
                self.knife_reset_task.cancel()
                self.knife_reset_task = None
            except:
                pass
        await self.field_friend.z_axis.turn_knifes_to(0)

    async def prepare(self) -> None:
        res = await super().prepare()
        if self.y_axis_move_task:
            try:
                self.y_axis_move_task.cancel()
                self.y_axis_move_task = None
            except:
                res = False
        if self.knife_reset_task:
            try:
                self.knife_reset_task.cancel()
                self.knife_reset_task = None
            except:
                res = False
        return res

    async def start_workflow(self) -> None:
        await super().start_workflow()
        self.log.info('Performing Tornado Workflow..')
        try:
            # TODO: do we need to set self.next_crop_id = '' on every return?
            punch_position = self.system.odometer.prediction.transform(
                rosys.geometry.Point(x=self.system.field_friend.WORK_X, y=self.next_punch_y_position))
            self.last_punches.append(punch_position)
            self.log.info(f'Drilling crop at {punch_position} with angle {self.tornado_angle}°')
            open_drill = False
            if self.drill_with_open_tornado:
                open_drill = True

            if self.y_axis_move_task:
                await self.y_axis_move_task
                if self.y_axis_move_task.exception():
                    raise self.y_axis_move_task.exception()
                self.y_axis_move_task = None

            if self.knife_reset_task:
                await self.knife_reset_task
                if self.knife_reset_task.exception():
                    raise self.knife_reset_task.exception()
                self.knife_reset_task = None

            await self.system.puncher.punch(y=self.next_punch_y_position, angle=self.tornado_angle, with_open_tornado=open_drill)
            if self.move_knifes_while_driving:
                self.knife_reset_task = asyncio.create_task(self._reset_knifes())
            else:
                await self._reset_knifes()
            # TODO remove weeds from plant_provider and increment kpis (like in Weeding Screw)
            if isinstance(self.system.detector, rosys.vision.DetectorSimulation):
                # remove the simulated weeds
                inner_radius = 0.025  # TODO compute inner radius according to tornado angle
                outer_radius = inner_radius + 0.05  # TODO compute outer radius according to inner radius and knife width
                # inner_diameter, outer_diameter = self.system.field_friend.tornado_diameters(self.tornado_angle)
                # inner_radius = inner_diameter / 2
                # outer_radius = outer_diameter / 2
                self.system.detector.simulated_objects = [obj for obj in self.system.detector.simulated_objects
                                                          if not (inner_radius <= obj.position.projection().distance(punch_position) <= outer_radius)]
                self.log.info(f'simulated_objects2: {len(self.system.detector.simulated_objects)}')
        except PuncherException:
            self.log.error('Error in Tornado Workflow')
        except Exception as e:
            raise ImplementException('Error while tornado Workflow') from e

    async def get_stretch(self, max_distance: float) -> float:
        await super().get_stretch(max_distance)
        super()._has_plants_to_handle()
        if len(self.crops_to_handle) == 0:
            return self.WORKING_DISTANCE
        closest_crop_id, closest_crop_position = list(self.crops_to_handle.items())[0]
        closest_crop_world_position = self.system.odometer.prediction.transform(closest_crop_position)

        # for p in self.last_punches:
        #     self.log.info(f'Last punch: {p} - {p.distance(closest_crop_world_position)} - {self.crop_safety_distance} - {closest_crop_world_position}')
        if any(p.distance(closest_crop_world_position) < self.field_friend.DRILL_RADIUS for p in self.last_punches):
            self.log.info('Skipping crop because it was already punched')
            return self.WORKING_DISTANCE
        if not self.system.field_friend.can_reach(closest_crop_position):
            self.log.info('Target crop is not reachable')
            return self.WORKING_DISTANCE
        if closest_crop_position.x >= self.system.field_friend.WORK_X + self.WORKING_DISTANCE:
            self.log.info('Closest crop is out of working area')
            return self.WORKING_DISTANCE
        if self._crops_in_drill_range(closest_crop_id, closest_crop_position, self.tornado_angle):
            self.log.info('Crops in drill range')
            return self.WORKING_DISTANCE

        stretch = closest_crop_position.x - self.system.field_friend.WORK_X
        if stretch < - self.system.field_friend.DRILL_RADIUS:
            self.log.info(f'Skipping crop {closest_crop_id} because it is behind the robot')
            return self.WORKING_DISTANCE
        if stretch < 0:
            stretch = 0
        if stretch < max_distance:
            self.next_punch_y_position = closest_crop_position.y
            if self.move_y_while_driving:
                self.y_axis_move_task = asyncio.create_task(self.field_friend.y_axis.move_to(self.next_punch_y_position))
            return stretch
        return self.WORKING_DISTANCE

    def _crops_in_drill_range(self, crop_id: str, crop_position: rosys.geometry.Point, angle: float) -> bool:
        inner_diameter, outer_diameter = self.system.field_friend.tornado_diameters(angle)
        crop_world_position = self.system.odometer.prediction.transform(crop_position)
        for crop in self.system.plant_provider.crops:
            if crop.id != crop_id:
                distance = crop_world_position.distance(crop.position)
                if distance >= inner_diameter/2 and distance <= outer_diameter/2:
                    return True
        return False

    def backup(self) -> dict:
        return super().backup() | {
            'drill_with_open_tornado': self.drill_with_open_tornado,
            'drill_between_crops': self.drill_between_crops,
            'tornado_angle': self.tornado_angle,
            'is_demo': self.puncher.is_demo,
        }

    def restore(self, data: dict[str, Any]) -> None:
        super().restore(data)
        self.drill_with_open_tornado = data.get('drill_with_open_tornado', self.drill_with_open_tornado)
        self.drill_between_crops = data.get('drill_between_crops', self.drill_between_crops)
        self.tornado_angle = data.get('tornado_angle', self.tornado_angle)
        self.puncher.is_demo = data.get('is_demo', self.puncher.is_demo)

    def settings_ui(self):
        super().settings_ui()
        ui.number('Tornado angle', format='%.0f', step=1, min=0, max=180) \
            .props('dense outlined suffix=°') \
            .classes('w-24') \
            .bind_value(self, 'tornado_angle') \
            .tooltip('Set the angle for the tornado drill')
        ui.label().bind_text_from(self, 'tornado_angle', lambda v: f'Tornado diameters: {self.field_friend.tornado_diameters(v)[0]*100:.1f} cm '
                                  f'- {self.field_friend.tornado_diameters(v)[1]*100:.1f} cm')
        ui.checkbox('Demo Mode') \
            .bind_value(self.puncher, 'is_demo') \
            .tooltip('If active, stop right before the ground')
        
        ui.checkbox("Move y while driving").bind_value(self, 'move_y_while_driving').tooltip("Moves the y-axis into position whilst driving.")
        ui.checkbox("Move knifes while driving").bind_value(self, 'move_knifes_while_driving').tooltip("Moves the knifes into the correct angle whilst driving.")
        # TODO test and reactivate these options
        # ui.checkbox('Drill 2x with open tornado') \
        #     .bind_value(self, 'drill_with_open_tornado') \
        #     .tooltip('Set the weeding automation to drill a second time with open tornado')
        # ui.checkbox('Drill between crops') \
        #     .bind_value(self, 'drill_between_crops') \
        #     .tooltip('Set the weeding automation to drill between crops')
