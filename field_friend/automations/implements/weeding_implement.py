import logging
from collections import deque
from typing import TYPE_CHECKING, Any

import rosys
from nicegui import ui
from rosys.analysis import track
from rosys.geometry import Point3d, Pose

from ...hardware import Axis, ChainAxis, Sprayer, Tornado
from .implement import Implement

if TYPE_CHECKING:
    from ...system import System


class ImplementException(Exception):
    pass


class WeedingImplement(Implement):
    FLASHLIGHT_WAIT_TIME = 3.0
    LOCATOR_WAIT_TIME = 5.0

    def __init__(self,  name: str, system: 'System') -> None:
        super().__init__(name)
        self.relevant_weeds = system.plant_locator.weed_category_names
        self.log = logging.getLogger('field_friend.weeding')
        self.system = system
        self.puncher = system.puncher
        self.record_video = False
        self.cultivated_crop_select: ui.select | None = None
        self.cultivated_crop: str | None = None
        self.crop_safety_distance: float = 0.01

        # dual mechanism
        self.with_drilling: bool = False
        self.with_chopping: bool = False
        self.chop_if_no_crops: bool = False

        self.start_time: float | None = None
        self.last_pose: Pose | None = None
        self.driven_distance: float = 0.0
        self.crops_to_handle: dict[str, Point3d] = {}
        self.weeds_to_handle: dict[str, Point3d] = {}
        self.last_punches: deque[Point3d] = deque(maxlen=5)
        self.next_punch_y_position: float = 0

    async def prepare(self) -> bool:
        await super().prepare()
        if self.system.plant_locator.detector_info is None and not await self.system.plant_locator.fetch_detector_info():
            rosys.notify('Dectection model information not available', 'negative')
            return False
        if self.cultivated_crop_select is not None:
            self.log.debug('setting cultivated crop options')
            self.cultivated_crop_select.set_options(self.system.plant_locator.crop_category_names)
        self.log.info(f'start weeding {self.relevant_weeds} with {self.name} ...')
        self.request_backup()
        if not await self._check_hardware_ready():
            rosys.notify('hardware is not ready')
            return False
        self.last_punches.clear()
        await rosys.sleep(2.0)  # NOTE: _check_hardware_ready sometimes returns too early
        return True

    async def finish(self) -> None:
        self.system.plant_locator.pause()
        await self.system.field_friend.stop()
        try:
            await self.puncher.clear_view()
        except Exception as e:
            self.log.error(f'Error clearing view: {e}')
        await self.system.timelapse_recorder.compress_video()
        await super().finish()

    async def activate(self):
        if self.system.field_friend.flashlight:
            await self.system.field_friend.flashlight.turn_on()
        await self.puncher.clear_view()
        await rosys.sleep(self.FLASHLIGHT_WAIT_TIME)
        self.system.plant_locator.resume()
        await rosys.sleep(self.LOCATOR_WAIT_TIME)
        assert self.system.camera_provider is not None
        if self.record_video:
            self.system.timelapse_recorder.camera = self.system.camera_provider.first_connected_camera
        await super().activate()

    async def deactivate(self):
        await super().deactivate()
        self.system.timelapse_recorder.camera = None
        if self.system.field_friend.flashlight:
            await self.system.field_friend.flashlight.turn_off()
        self.system.plant_locator.pause()

    async def start_workflow(self) -> None:
        # TODO: only sleep when moving
        await rosys.sleep(2)  # wait for robot to stand still
        if not self.has_plants_to_handle():
            return
        self.log.debug(f'Handling plants with {self.name}...')

    async def stop_workflow(self) -> None:
        self.log.debug('workflow completed')
        self.crops_to_handle = {}
        self.weeds_to_handle = {}

    @track
    async def _check_hardware_ready(self) -> bool:
        if self.system.field_friend.estop.active or self.system.field_friend.estop.is_soft_estop_active:
            rosys.notify('E-Stop is active, aborting', 'negative')
            return False
        if self.system.camera_provider is None:
            rosys.notify('no camera provider configured')
            return False
        camera = self.system.camera_provider.first_connected_camera
        if not camera:
            rosys.notify('no camera connected')
            return False
        if hasattr(camera, 'calibration') and camera.calibration is None:
            rosys.notify('camera has no calibration')
            return False

        z_axis = self.system.field_friend.z_axis
        if isinstance(z_axis, Axis | Tornado | Sprayer):
            if isinstance(z_axis, Axis) and z_axis.alarm:
                rosys.notify('Z-Axis is in alarm, aborting', 'negative')
                return False
            if not z_axis.is_referenced and not await z_axis.try_reference():
                rosys.notify('Referencing Z-Axis failed, aborting', 'negative')
                return False
        y_axis = self.system.field_friend.y_axis
        if isinstance(y_axis, Axis | ChainAxis):
            if y_axis.alarm:
                rosys.notify('Y-Axis is in alarm, aborting', 'negative')
                return False
            if not y_axis.is_referenced and not await y_axis.try_reference():
                rosys.notify('Referencing Y-Axis failed, aborting', 'negative')
                return False
        return True

    def has_plants_to_handle(self) -> bool:
        current_pose = self.system.robot_locator.pose
        relative_crop_positions = {
            c.id: Point3d.from_point(current_pose.relative_point(c.position.projection()))
            for c in self.system.plant_provider.get_relevant_crops(current_pose.point_3d())
            if self.cultivated_crop is None or c.type == self.cultivated_crop
        }
        upcoming_crop_positions = {
            c: pos for c, pos in relative_crop_positions.items()
            if self.system.field_friend.WORK_X - self.system.field_friend.DRILL_RADIUS < pos.x < 0.3
        }
        # Sort the upcoming positions so nearest comes first
        sorted_crops = dict(sorted(upcoming_crop_positions.items(), key=lambda item: item[1].x))
        self.crops_to_handle = sorted_crops

        relative_weed_positions = {
            w.id: Point3d.from_point(current_pose.relative_point(w.position.projection()))
            for w in self.system.plant_provider.get_relevant_weeds(current_pose.point_3d())
            if w.type in self.relevant_weeds
        }
        upcoming_weed_positions = {
            w: pos for w, pos in relative_weed_positions.items()
            if self.system.field_friend.WORK_X - self.system.field_friend.DRILL_RADIUS < pos.x < 0.4
        }

        # keep crops safe by pushing weeds away so the implement does not accidentally hit a crop
        for _, crop_position in sorted_crops.items():
            for weed, weed_position in upcoming_weed_positions.items():
                offset = self.system.field_friend.DRILL_RADIUS + \
                    self.crop_safety_distance - crop_position.distance(weed_position)
                if offset > 0:
                    # TODO: check if this is correct
                    weed_position_2d = weed_position.projection()
                    crop_position_2d = crop_position.projection()
                    safe_weed_position_2d = weed_position_2d.polar(offset, crop_position_2d.direction(weed_position_2d))
                    upcoming_weed_positions[weed] = Point3d.from_point(safe_weed_position_2d)
                    self.log.debug('corrected weed position: %s -> %s', weed_position_2d, safe_weed_position_2d)

        # Sort the upcoming positions so nearest comes first
        sorted_weeds = dict(sorted(upcoming_weed_positions.items(), key=lambda item: item[1].x))
        self.weeds_to_handle = sorted_weeds
        return False

    def backup_to_dict(self) -> dict[str, Any]:
        return super().backup_to_dict() | {
            'with_drilling': self.with_drilling,
            'with_chopping': self.with_chopping,
            'chop_if_no_crops': self.chop_if_no_crops,
            'cultivated_crop': self.cultivated_crop,
            'crop_safety_distance': self.crop_safety_distance,
            'record_video': self.record_video,
            'is_demo': self.puncher.is_demo,
        }

    def restore_from_dict(self, data: dict[str, Any]) -> None:
        super().restore_from_dict(data)
        self.with_drilling = data.get('with_drilling', self.with_drilling)
        self.with_chopping = data.get('with_chopping', self.with_chopping)
        self.chop_if_no_crops = data.get('chop_if_no_crops', self.chop_if_no_crops)
        self.cultivated_crop = data.get('cultivated_crop', self.cultivated_crop)
        self.crop_safety_distance = data.get('crop_safety_distance', self.crop_safety_distance)
        self.record_video = data.get('record_video', self.record_video)
        self.puncher.is_demo = data.get('is_demo', self.puncher.is_demo)

    def clear(self) -> None:
        self.crops_to_handle = {}
        self.weeds_to_handle = {}

    def settings_ui(self) -> None:
        super().settings_ui()

        async def update_cultivated_crop() -> None:
            assert self.cultivated_crop_select is not None
            if self.cultivated_crop_select.options:
                self.log.debug('crop options already set')
                return
            await self.system.plant_locator.fetch_detector_info()
            self.cultivated_crop_select.set_options(self.system.plant_locator.crop_category_names)

        self.cultivated_crop_select = ui.select(self.system.plant_locator.crop_category_names, label='cultivated crop', on_change=self.request_backup) \
            .bind_value(self, 'cultivated_crop').props('clearable') \
            .classes('w-40').tooltip('Set the cultivated crop which should be kept safe') \
            .on('mouseenter', update_cultivated_crop)
        ui.number('Crop safety distance', step=0.001, min=0.001, max=0.05, format='%.3f', on_change=self.request_backup) \
            .props('dense outlined suffix=m') \
            .classes('w-24') \
            .bind_value(self, 'crop_safety_distance') \
            .tooltip('Set the crop safety distance for the weeding automation')
        ui.checkbox('record video', on_change=self.request_backup) \
            .bind_value(self, 'record_video') \
            .tooltip('Set the weeding automation to record video')
        ui.checkbox('Demo Mode', on_change=self.request_backup) \
            .bind_value(self.puncher, 'is_demo') \
            .tooltip('If active, stop right before the ground')
