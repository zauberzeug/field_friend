from __future__ import annotations

from typing import TYPE_CHECKING, Any

import rosys
from nicegui import ui
from rosys.analysis import track
from rosys.geometry import Point

from .weeding_implement import ImplementException, WeedingImplement

if TYPE_CHECKING:
    from ...system import System


class WeedingSprayer(WeedingImplement):
    PRESSURE_REACH_TIME = 10.0
    SPRAY_TIME = 0.5

    def __init__(self, system: System) -> None:
        super().__init__('Sprayer', system)
        self.sprayer_hardware = system.field_friend.z_axis
        self.relevant_weeds = ['coix', 'sauerampfer', 'weed']
        self.log.debug('Using relevant weeds: %s', self.relevant_weeds)

        self.pressure_reach_time: float = self.PRESSURE_REACH_TIME
        self.spray_time: float = self.SPRAY_TIME

    async def prepare(self) -> bool:
        await self.system.field_friend.z_axis.activate_pump()
        await rosys.sleep(self.pressure_reach_time)
        return True

    async def start_workflow(self) -> None:
        await super().start_workflow()
        try:
            punch_position = self.system.robot_locator.pose.transform3d(
                rosys.geometry.Point3d(x=self.system.field_friend.WORK_X, y=0.0, z=0))
            self.last_punches.append(punch_position)
            await self.system.field_friend.z_axis.open_valve()
            await rosys.sleep(self.spray_time)
            await self.system.field_friend.z_axis.close_valve()
            punched_weeds = [weed.id for weed in self.system.plant_provider.get_relevant_weeds(self.system.robot_locator.pose.point_3d())
                             if weed.position.distance(punch_position) <= self.sprayer_hardware.spray_radius]
            punched_crops = [crop.id for crop in self.system.plant_provider.get_relevant_crops(self.system.robot_locator.pose.point_3d())
                             if crop.position.distance(punch_position) <= self.sprayer_hardware.spray_radius]
            for weed_id in punched_weeds:
                self.system.plant_provider.remove_weed(weed_id)
            for crop_id in punched_crops:
                self.system.plant_provider.remove_crop(crop_id)
            if isinstance(self.system.detector, rosys.vision.DetectorSimulation):
                self.system.detector.simulated_objects = [
                    obj for obj in self.system.detector.simulated_objects
                    if obj.position.projection().distance(punch_position.projection()) > self.sprayer_hardware.spray_radius]
        except Exception as e:
            raise ImplementException(f'Error in Weed Spray Workflow: {e}') from e


    @track
    async def get_move_target(self) -> Point | None:  # pylint: disable=unused-argument
        """Return the target position to drive to."""
        super()._has_plants_to_handle()
        weeds_in_range = {weed_id: position for weed_id, position in self.weeds_to_handle.items()
                          if self.system.field_friend.can_reach(position.projection())}
        if not weeds_in_range:
            self.log.debug('No weeds in range')
            return None
        self.log.debug(f'Found {len(weeds_in_range)} weeds in range: {weeds_in_range}')
        for next_weed_id, next_weed_position in weeds_in_range.items():
            weed_world_position = self.system.robot_locator.pose.transform3d(next_weed_position)
            # crops = self.system.plant_provider.get_relevant_crops(self.system.robot_locator.pose.point_3d())
            # if self.cultivated_crop and not any(c.position.distance(weed_world_position) < self.max_crop_distance for c in crops):
            #     self.log.debug('Skipping weed because it is to far from the cultivated crops')
            #     continue
            if any(p.distance(weed_world_position) < self.sprayer_hardware.spray_radius for p in self.last_punches):
                self.log.debug('Skipping weed because it was already punched')
                continue
            relative_x = next_weed_position.x - self.system.field_friend.WORK_X
            if relative_x < - self.sprayer_hardware.spray_radius:
                self.log.debug(f'Skipping weed {next_weed_id} because it is behind the robot')
                continue
            self.log.debug('Targeting weed %s which is %s away at world: %s, local: %s', next_weed_id, relative_x, weed_world_position, next_weed_position)
            self.next_punch_y_position = next_weed_position.y
            return weed_world_position.projection()
        return None

    async def finish(self) -> None:
        # TODO: stop pump
        await super().finish()

    def backup_to_dict(self) -> dict[str, Any]:
        return super().backup_to_dict() | {
            'pressure_reach_time': self.pressure_reach_time,
            'spray_time': self.spray_time,
        }

    def restore_from_dict(self, data: dict[str, Any]) -> None:
        super().restore_from_dict(data)
        self.pressure_reach_time = data.get('pressure_reach_time', self.pressure_reach_time)
        self.spray_time = data.get('spray_time', self.spray_time)

    def settings_ui(self) -> None:
        super().settings_ui()
        ui.number('Pressure reach time', step=1.0, min=0.0, max=60.0, format='%.1f', on_change=self.request_backup) \
            .props('dense outlined suffix=s') \
            .classes('w-24') \
            .bind_value(self, 'pressure_reach_time') \
            .tooltip('Set the time to wait until the pump has reached the pressure')
        ui.number('Spray time', step=0.01, min=0.0, max=10.0, format='%.1f', on_change=self.request_backup) \
            .props('dense outlined suffix=s') \
            .classes('w-24') \
            .bind_value(self, 'spray_time') \
            .tooltip('Set the time to spray')
