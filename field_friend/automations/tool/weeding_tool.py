import logging
from random import randint
from typing import TYPE_CHECKING, Any, Optional

import numpy as np
import rosys
from nicegui import ui
from rosys.driving import PathSegment
from rosys.geometry import Point, Pose, Spline
from rosys.helpers import eliminate_2pi

from ...hardware import ChainAxis
from ..navigation import Navigation
from ..plant import Plant
from ..tool.tool import Tool

if TYPE_CHECKING:
    from system import System


class ToolException(Exception):
    pass


class WeedingTool(Tool, rosys.persistence.PersistentModule):
    WORKING_DISTANCE = 0.06
    DRIVE_DISTANCE = 0.04

    def __init__(self,  name: str, system: 'System', persistence_key: str = 'weeding') -> None:
        Tool.__init__(self, name)
        rosys.persistence.PersistentModule.__init__(self, persistence_key=f'field_friend.automations.{persistence_key}')

        self.log = logging.getLogger('field_friend.weeding')
        self.system = system
        self.kpi_provider = system.kpi_provider

        # dual mechanism
        self.with_drilling: bool = False
        self.with_chopping: bool = False
        self.chop_if_no_crops: bool = False

        # tool settings
        self.tornado_angle: float = 30.0
        self.weed_screw_depth: float = 0.13
        self.crop_safety_distance: float = 0.01

        self.state: str = 'idle'
        self.start_time: Optional[float] = None
        self.last_pose: Optional[Pose] = None
        self.drived_distance: float = 0.0
        self.crops_to_handle: dict[str, Point] = {}
        self.weeds_to_handle: dict[str, Point] = {}

        rosys.on_repeat(self._update_time_and_distance, 0.1)
        self.system.field_provider.FIELD_SELECTED.register(self.clear)

    async def activate(self):
        self.system.plant_locator.pause()
        self.system.plant_provider.clear()
        if self.system.field_friend.tool != 'none':
            await self.system.puncher.clear_view()
        await self.system.field_friend.flashlight.turn_on()
        await rosys.sleep(3)
        self.system.plant_locator.resume()
        await rosys.sleep(3)
        # if not self.system.is_real:
        #     self.system.detector.simulated_objects = []
        #     await self._create_simulated_plants()

    async def deactivate(self):
        await self.system.field_friend.flashlight.turn_off()
        self.system.plant_locator.pause()
        self.kpi_provider.increment_weeding_kpi('rows_weeded')

    def _update_time_and_distance(self):
        if self.state == 'idle':
            return
        if self.start_time is None:
            self.start_time = rosys.time()
        if self.last_pose is None:
            self.last_pose = self.system.odometer.prediction
            self.drived_distance = 0.0
        self.drived_distance += self.system.odometer.prediction.distance(self.last_pose)
        if self.drived_distance > 1:
            self.kpi_provider.increment_weeding_kpi('distance')
            self.drived_distance -= 1
        self.last_pose = self.system.odometer.prediction
        passed_time = rosys.time() - self.start_time
        if passed_time > 1:
            self.kpi_provider.increment_weeding_kpi('time')
            self.start_time = rosys.time()

    def backup(self) -> dict:
        return {
            'with_drilling': self.with_drilling,
            'with_chopping': self.with_chopping,
            'chop_if_no_crops': self.chop_if_no_crops,
            'tornado_angle': self.tornado_angle,
            'weed_screw_depth': self.weed_screw_depth,
            'crop_safety_distance': self.crop_safety_distance,
        }

    def restore(self, data: dict[str, Any]) -> None:
        self.with_drilling = data.get('with_drilling', self.with_drilling)
        self.with_chopping = data.get('with_chopping', self.with_chopping)
        self.chop_if_no_crops = data.get('chop_if_no_crops', self.chop_if_no_crops)
        self.tornado_angle = data.get('tornado_angle', self.tornado_angle)
        self.weed_screw_depth = data.get('weed_screw_depth', self.weed_screw_depth)
        self.crop_safety_distance = data.get('crop_safety_distance', self.crop_safety_distance)

    def clear(self) -> None:
        self.crops_to_handle = {}
        self.weeds_to_handle = {}

    async def prepare(self) -> bool:
        self.log.info('start weeding...')
        self.request_backup()
        if not await self._check_hardware_ready():
            rosys.notify('hardware is not ready')
            return False
        self.state = 'running'
        return True

    async def finish(self) -> None:
        self.system.plant_locator.pause()
        self.system.automation_watcher.stop_field_watch()
        self.system.automation_watcher.gnss_watch_active = False
        await self.system.field_friend.stop()

    async def _check_hardware_ready(self) -> bool:
        if self.system.field_friend.estop.active or self.system.field_friend.estop.is_soft_estop_active:
            rosys.notify('E-Stop is active, aborting', 'negative')
            self.log.error('E-Stop is active, aborting')
            return False
        camera = next((camera for camera in self.system.usb_camera_provider.cameras.values() if camera.is_connected), None)
        if not camera:
            rosys.notify('no camera connected')
            return False
        if camera.calibration is None:
            rosys.notify('camera has no calibration')
            return False
        if self.system.field_friend.tool == 'none':
            rosys.notify('This field friend has no tool, only monitoring', 'info')
            self.log.info('This field friend has no tool, only monitoring')
            return True
        if self.system.field_friend.y_axis.alarm:
            rosys.notify('Y-Axis is in alarm, aborting', 'negative')
            self.log.error('Y-Axis is in alarm, aborting')
            return False
        if isinstance(self.system.field_friend.y_axis, ChainAxis):
            if not self.system.field_friend.y_axis.ref_t:
                rosys.notify('ChainAxis is not in top ref', 'negative')
                self.log.error('ChainAxis is not in top ref')
                return False
        if not await self.system.puncher.try_home():
            rosys.notify('Puncher homing failed, aborting', 'negative')
            self.log.error('Puncher homing failed, aborting')
            return False
        return True

    async def observe(self) -> None:
        self.log.info('Checking for plants...')
        while True:
            if self._has_plants_to_handle():
                return
            await rosys.sleep(0.2)

    async def on_focus(self) -> None:
        await rosys.sleep(2)  # wait for robot to stand still
        if not self._has_plants_to_handle():
            return
        self.log.info(f'Handling plants with {self.name}...')
        await self._perform_workflow()
        self.crops_to_handle = {}
        self.weeds_to_handle = {}

    def _has_plants_to_handle(self) -> bool:
        relative_crop_positions = {
            c.id: self.system.odometer.prediction.relative_point(c.position)
            for c in self.system.plant_provider.crops if c.position.distance(self.system.odometer.prediction.point) < 0.5 and len(c.positions) >= 3
        }

        upcoming_crop_positions = {
            c: pos for c, pos in relative_crop_positions.items()
            if self.system.field_friend.WORK_X + self.system.field_friend.DRILL_RADIUS < pos.x < 0.3
        }
        # Sort the upcoming_crop_positions dictionary by the .x attribute of its values
        sorted_crops = dict(sorted(upcoming_crop_positions.items(), key=lambda item: item[1].x))
        self.crops_to_handle = sorted_crops

        relative_weed_positions = {
            w.id: self.system.odometer.prediction.relative_point(w.position)
            for w in self.system.plant_provider.weeds if w.position.distance(self.system.odometer.prediction.point) < 0.5 and len(w.positions) >= 3
        }
        upcoming_weed_positions = {
            w: pos for w, pos in relative_weed_positions.items()
            if self.system.field_friend.WORK_X+self.system.field_friend.DRILL_RADIUS < pos.x < 0.4
        }
        # Sort the upcoming_weed_positions dictionary by the .x attribute of its values
        sorted_weeds = dict(sorted(upcoming_weed_positions.items(), key=lambda item: item[1].x))
        self.weeds_to_handle = sorted_weeds
        return False

    async def _perform_workflow(self) -> None:
        pass

    async def _follow_line_of_crops(self):
        self.log.info('Following line of crops...')
        farthest_crop = list(self.crops_to_handle.values())[-1]
        self.log.info(f'Farthest crop: {farthest_crop}')
        upcoming_world_position = self.system.odometer.prediction.transform(farthest_crop)
        yaw = self.system.odometer.prediction.point.direction(upcoming_world_position)
        # only apply minimal yaw corrections to avoid oversteering
        target_yaw = self._weighted_angle_combine(self.system.odometer.prediction.yaw, 0.85, yaw, 0.15)
        # yaw = eliminate_2pi(self.system.odometer.prediction.yaw) * 0.9 + eliminate_2pi(yaw) * 0.1
        target = self.system.odometer.prediction.point.polar(self.DRIVE_DISTANCE, target_yaw)
        self.log.info(f'Current world position: {self.system.odometer.prediction} Target next crop at {target}')
        await self.system.driver.drive_to(target)

    def _weighted_angle_combine(self, angle1: float, weight1: float, angle2: float, weight2: float) -> float:
        # Normalize both angles
        angle1 = eliminate_2pi(angle1)
        angle2 = eliminate_2pi(angle2)

        # Combine angles with the weights
        x = np.cos(angle1) * weight1 + np.cos(angle2) * weight2
        y = np.sin(angle1) * weight1 + np.sin(angle2) * weight2

        # Compute the resultant angle
        combined_angle = np.arctan2(y, x)

        # Normalize the resultant angle
        return eliminate_2pi(combined_angle)

    async def _driving_a_bit_forward(self):
        self.log.info('No crops and no weeds in range, driving forward a bit...')
        target = self.system.odometer.prediction.point.polar(self.DRIVE_DISTANCE, self.system.odometer.prediction.yaw)
        self.log.info(f'Current world position: {self.system.odometer.prediction} Target: {target}')
        await self.system.driver.drive_to(target)

    def _keep_crops_safe(self) -> None:
        self.log.info('Keeping crops safe...')
        for crop in self.system.plant_provider.crops:
            crop_position = self.system.odometer.prediction.transform(crop.position)
            for weed, weed_position in self.weeds_to_handle.items():
                offset = self.system.field_friend.DRILL_RADIUS + \
                    self.crop_safety_distance - crop.position.distance(weed_position)
                if offset > 0:
                    safe_weed_position = weed_position.polar(offset, crop_position.direction(weed_position))
                    self.weeds_to_handle[weed] = safe_weed_position
                    self.log.info(
                        f'Moved weed {weed} from {weed_position} to {safe_weed_position} by {offset} to safe {crop.id} at {crop_position}')

    def _crops_in_drill_range(self, crop_id: str, crop_position: Point, angle: float) -> bool:
        inner_diameter, outer_diameter = self.system.field_friend.tornado_diameters(angle)
        for crop in self.system.plant_provider.crops:
            crop_world_position = self.system.odometer.prediction.transform(crop_position)
            if crop.id != crop_id:
                distance = crop_world_position.distance(crop.position)
                if distance >= inner_diameter/2 and distance <= outer_diameter/2:
                    return True
        return False

    async def _create_simulated_plants(self):
        self.log.info('Creating simulated plants...')
        if self.current_segment:
            self.log.info('Creating simulated plants for current segment')
            distance = self.current_segment.spline.start.distance(self.current_segment.spline.end)
            for i in range(1, int(distance/0.20)):
                for j in range(1, 4):
                    await self.system.plant_provider.add_crop(Plant(
                        id_=f'{i}_{j}',
                        type_='beet',
                        position=self.system.odometer.prediction.point.polar(
                            0.14*i, self.system.odometer.prediction.yaw).polar(randint(-2, 2)*0.01, self.system.odometer.prediction.yaw+np.pi/2),
                        detection_time=rosys.time(),
                        confidence=0.9,
                    ))
                    # for j in range(1, 5):
                    #     await self.system.plant_provider.add_weed(Plant(
                    #         id=f'{i}_{j}',
                    #         type='weed',
                    #         positions=[self.system.odometer.prediction.point.polar(
                    #             0.20*i+randint(-5, 5)*0.01, self.system.odometer.prediction.yaw).polar(randint(-15, 15)*0.01, self.system.odometer.prediction.yaw + np.pi/2)],
                    #         detection_time=rosys.time(),
                    #         confidence=0.9,
                    #     ))
        else:
            self.log.info('Creating simulated plants for whole row')
            for i in range(0, 30):
                await self.system.plant_provider.add_crop(Plant(
                    id_=str(i),
                    type_='beet',
                    position=self.system.odometer.prediction.point.polar(0.20*i,
                                                                         self.system.odometer.prediction.yaw)
                    .polar(randint(-2, 2)*0.01, self.system.odometer.prediction.yaw+np.pi/2),
                    detection_time=rosys.time(),
                    confidence=0.9,
                ))
                for j in range(1, 7):
                    await self.system.plant_provider.add_weed(Plant(
                        id_=f'{i}_{j}',
                        type_='weed',
                        position=self.system.odometer.prediction.point.polar(0.20*i+randint(-5, 5)*0.01,
                                                                             self.system.odometer.prediction.yaw)
                        .polar(randint(-15, 15)*0.01, self.system.odometer.prediction.yaw + np.pi/2),
                        detection_time=rosys.time(),
                        confidence=0.9,
                    ))

    def settings_ui(self):
        super().settings_ui()
        ui.markdown('Field settings').style('color: #6E93D6')
        with ui.row():
            with_field_planning = ui.checkbox('Use field planning', value=True) \
                .bind_value(self, 'use_field_planning') \
                .tooltip('Set the weeding automation to use the field planning with GNSS')

            with ui.row().bind_visibility_from(with_field_planning, 'value', value=True):
                self.show_start_row()
                self.show_end_row()

                ui.number('Min. turning radius', format='%.2f',
                          value=0.5, step=0.05, min=0.05, max=2.0) \
                    .props('dense outlined suffix=m').classes('w-30') \
                    .bind_value(self.system.weeding, 'minimum_turning_radius') \
                    .tooltip('Set the turning radius for the weeding automation')
                ui.number('turn_offset', format='%.2f', value=0.4, step=0.05, min=0.05, max=2.0) \
                    .props('dense outlined suffix=m').classes('w-30') \
                    .bind_value(self.system.weeding, 'turn_offset') \
                    .tooltip('Set the turning offset for the weeding automation')
                ui.checkbox('Drive backwards to start', value=True).bind_value(self.system.weeding, 'drive_backwards_to_start') \
                    .tooltip('Set the weeding automation to drive backwards to the start row at the end of the row')
                ui.checkbox('Drive to start row', value=True).bind_value(self.system.weeding, 'drive_to_start') \
                    .tooltip('Set the weeding automation to drive to the start of the row before starting the weeding')

    @ui.refreshable
    def show_start_row(self) -> None:
        if self.system.field_provider.active_field is not None:
            ui.select({row.id: row.name for row in self.system.field_provider.active_field.rows}, label='Start row') \
                .bind_value(self.system.weeding, 'start_row_id').classes('w-24').tooltip('Select the row to start on')
        else:
            ui.select([None], label='Start row')\
                .bind_value(self.system.weeding, 'start_row').classes('w-24').tooltip('Select the row to start on')

    @ui.refreshable
    def show_end_row(self) -> None:
        if self.system.field_provider.active_field is not None:
            ui.select({row.id: row.name for row in self.system.field_provider.active_field.rows}, label='End row') \
                .bind_value(self.system.weeding, 'end_row_id').classes('w-24').tooltip('Select the row to end on')
        else:
            ui.select([None], label='End row') \
                .bind_value(self.system.weeding, 'end_row').classes('w-24').tooltip('Select the row to end on')

    def reset_kpis(self):
        super().reset_kpis()
        self.kpi_provider.clear_weeding_kpis()
