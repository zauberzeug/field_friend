
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
        self.with_punch_check: bool = False
        self.field_friend = system.field_friend
        self.next_crop_id: str = ''
        self.punched_crops: deque[str] = deque(maxlen=20)

    async def start_workflow(self) -> bool:
        await super().start_workflow()
        self.log.info('Performing Tornado Workflow..')
        try:
            # TODO: do we need to set self.next_crop_id = '' on every return?
            if self.next_crop_id in self.punched_crops:
                self.log.info(f'crop {self.next_crop_id} already punched')
                return True
            if not self.next_crop_id in self.crops_to_handle:
                self.log.error('next crop not found in crops_to_handle')
                return True
            closest_crop_position = self.crops_to_handle[self.next_crop_id]
            target_world_position = self.system.odometer.prediction.transform(closest_crop_position)
            self.log.info(f'closest crop position: relative={closest_crop_position} world={target_world_position}')
            # fist check if the closest crop is in the working area
            if closest_crop_position.x >= self.system.field_friend.WORK_X + self.WORKING_DISTANCE:
                self.log.info('closest crop is out of working area')
                return True
            self.log.info(f'target next crop at {closest_crop_position}')
            if not self.system.field_friend.can_reach(closest_crop_position):
                self.log.info('target crop is not reachable')
                return True
            if self._crops_in_drill_range(self.next_crop_id, closest_crop_position, self.tornado_angle):
                self.log.info('crops in drill range')
                return True
            self.log.info('drilling crop')
            open_drill = False
            if self.drill_with_open_tornado and not self._crops_in_drill_range(self.next_crop_id, closest_crop_position, 0):
                open_drill = True
            await self.system.puncher.punch(y=closest_crop_position.y, plant_id=self.next_crop_id, angle=self.tornado_angle,
                                                        with_open_tornado=open_drill,
                                                        with_punch_check=self.with_punch_check)
            self.punched_crops.append(self.next_crop_id)
            self.next_crop_id = ''
            # TODO remove weeds from plant_provider and increment kpis (like in Weeding Screw)
            if isinstance(self.system.detector, rosys.vision.DetectorSimulation):
                # remove the simulated weeds
                inner_radius = 0.025  # TODO compute inner radius according to tornado angle
                outer_radius = inner_radius + 0.05  # TODO compute outer radius according to inner radius and knife width
                self.system.detector.simulated_objects = [obj for obj in self.system.detector.simulated_objects
                                                            if not (inner_radius <= obj.position.projection().distance(target_world_position) <= outer_radius)]

            return True
        except PuncherException:
            self.log.error('Error in Tornado Workflow')
            return True
        except Exception as e:
            raise ImplementException('Error while tornado Workflow') from e
    
    def get_stretch(self) -> float:
        super()._has_plants_to_handle()
        if len(self.crops_to_handle) == 0:
            return self.WORKING_DISTANCE
        self.crops_to_handle = {crop_id: crop_position for crop_id, crop_position in self.crops_to_handle.items() if crop_id not in self.punched_crops}
        if len(self.crops_to_handle) == 0:
            return self.WORKING_DISTANCE
        closest_crop_id, closest_crop_position = list(self.crops_to_handle.items())[0]
        self.next_crop_id = closest_crop_id
        return closest_crop_position.x - self.system.field_friend.WORK_X

    def _crops_in_drill_range(self, crop_id: str, crop_position: rosys.geometry.Point, angle: float) -> bool:
        inner_diameter, outer_diameter = self.system.field_friend.tornado_diameters(angle)
        for crop in self.system.plant_provider.crops:
            crop_world_position = self.system.odometer.prediction.transform(crop_position)
            if crop.id != crop_id:
                distance = crop_world_position.distance(crop.position)
                if distance >= inner_diameter/2 and distance <= outer_diameter/2:
                    return True
        return False

    def backup(self) -> dict:
        return super().backup() | {
            'drill_with_open_tornado': self.drill_with_open_tornado,
            'drill_between_crops': self.drill_between_crops,
            'with_punch_check': self.with_punch_check,
            'tornado_angle': self.tornado_angle,
        }

    def restore(self, data: dict[str, Any]) -> None:
        super().restore(data)
        self.drill_with_open_tornado = data.get('drill_with_open_tornado', self.drill_with_open_tornado)
        self.drill_between_crops = data.get('drill_between_crops', self.drill_between_crops)
        self.with_punch_check = data.get('with_punch_check', self.with_punch_check)
        self.tornado_angle = data.get('tornado_angle', self.tornado_angle)

    def settings_ui(self):
        super().settings_ui()

        ui.number('Tornado angle', format='%.0f', step=1, min=0, max=180) \
            .props('dense outlined suffix=°') \
            .classes('w-24') \
            .bind_value(self, 'tornado_angle') \
            .tooltip('Set the angle for the tornado drill')
        ui.label().bind_text_from(self, 'tornado_angle', lambda v: f'Tornado diameters: {self.field_friend.tornado_diameters(v)[0]*100:.1f} cm '
                                  f'- {self.field_friend.tornado_diameters(v)[1]*100:.1f} cm')
        ui.checkbox('With punch check') \
            .bind_value(self, 'with_punch_check') \
            .tooltip('Set the weeding automation to check for punch')
        ui.checkbox('Drill 2x with open torando') \
            .bind_value(self, 'drill_with_open_tornado') \
            .tooltip('Set the weeding automation to drill a second time with open tornado')
        ui.checkbox('Drill between crops') \
            .bind_value(self, 'drill_between_crops') \
            .tooltip('Set the weeding automation to drill between crops')
