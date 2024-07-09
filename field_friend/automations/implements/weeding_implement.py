import logging
from typing import TYPE_CHECKING, Any, Optional

import rosys
from nicegui import ui
from rosys.geometry import Point, Pose

from ...hardware import ChainAxis
from .implement import Implement

if TYPE_CHECKING:
    from system import System


class ImplementException(Exception):
    pass


class WeedingImplement(Implement, rosys.persistence.PersistentModule):
    WORKING_DISTANCE = 0.15

    def __init__(self,  name: str, system: 'System', persistence_key: str = 'weeding') -> None:
        Implement.__init__(self, name)
        rosys.persistence.PersistentModule.__init__(self,
                                                    persistence_key=f'field_friend.automations.implements.{persistence_key}')

        self.relevant_weeds = system.small_weed_category_names + system.big_weed_category_names
        self.log = logging.getLogger('field_friend.weeding')
        self.system = system
        self.kpi_provider = system.kpi_provider
        self.puncher = system.puncher
        self.cultivated_crop: str | None = None

        # dual mechanism
        self.with_drilling: bool = False
        self.with_chopping: bool = False
        self.chop_if_no_crops: bool = False

        self.state: str = 'idle'
        self.start_time: Optional[float] = None
        self.last_pose: Optional[Pose] = None
        self.driven_distance: float = 0.0
        self.crops_to_handle: dict[str, Point] = {}
        self.weeds_to_handle: dict[str, Point] = {}

        rosys.on_repeat(self._update_time_and_distance, 0.1)

    async def prepare(self) -> bool:
        await super().prepare()
        self.log.info(f'start weeding {self.relevant_weeds} with {self.name} ...')
        self.request_backup()
        if not await self._check_hardware_ready():
            rosys.notify('hardware is not ready')
            return False
        self.state = 'running'
        return True

    async def finish(self) -> None:
        self.system.plant_locator.pause()
        await self.system.field_friend.stop()
        await super().finish()

    async def activate(self):
        await super().activate()
        await self.system.field_friend.flashlight.turn_on()
        await self.puncher.clear_view()
        self.system.plant_locator.resume()
        await rosys.sleep(3)

    async def deactivate(self):
        await super().deactivate()
        await self.system.field_friend.flashlight.turn_off()
        self.system.plant_locator.pause()
        self.kpi_provider.increment_weeding_kpi('rows_weeded')

    async def observe(self) -> None:
        while self.get_stretch() > 0.05:
            await rosys.sleep(0.1)

    async def start_workflow(self) -> bool:
        await rosys.sleep(2)  # wait for robot to stand still
        if not self._has_plants_to_handle():
            return True
        self.log.info(f'Handling plants with {self.name}...')
        return True

    async def stop_workflow(self) -> None:
        self.log.info('workflow completed')
        self.crops_to_handle = {}
        self.weeds_to_handle = {}

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

    def _has_plants_to_handle(self) -> bool:
        relative_crop_positions = {
            c.id: self.system.odometer.prediction.relative_point(c.position)
            for c in self.system.plant_provider.get_relevant_crops(self.system.odometer.prediction.point)
            if self.cultivated_crop is None or c.type == self.cultivated_crop
        }
        upcoming_crop_positions = {
            c: pos for c, pos in relative_crop_positions.items()
            if self.system.field_friend.WORK_X < pos.x < 0.3
        }
        # Sort the upcoming positions so nearest comes first
        sorted_crops = dict(sorted(upcoming_crop_positions.items(), key=lambda item: item[1].x))
        self.crops_to_handle = sorted_crops

        relative_weed_positions = {
            w.id: self.system.odometer.prediction.relative_point(w.position)
            for w in self.system.plant_provider.get_relevant_weeds(self.system.odometer.prediction.point, max_distance=0.5)
            if w.type in self.relevant_weeds
        }
        upcoming_weed_positions = {
            w: pos for w, pos in relative_weed_positions.items()
            if self.system.field_friend.WORK_X < pos.x < 0.4
        }

        # TODO: is not gonna work for tornado because of crop_safety_distance
        for crop, crop_position in sorted_crops.items():
            for weed, weed_position in upcoming_weed_positions.items():
                offset = self.system.field_friend.DRILL_RADIUS + \
                    self.crop_safety_distance - crop_position.distance(weed_position)
                if offset > 0:
                    safe_weed_position = weed_position.polar(offset, crop_position.direction(weed_position))
                    upcoming_weed_positions[weed] = safe_weed_position
                    self.log.info(f'Moved weed {weed} from {weed_position} to {safe_weed_position} ' +
                                  f'by {offset} to safe {crop} at {crop_position}') 

        # Sort the upcoming positions so nearest comes first
        sorted_weeds = dict(sorted(upcoming_weed_positions.items(), key=lambda item: item[1].x))
        self.weeds_to_handle = sorted_weeds
        return False
    
    def reset_kpis(self):
        super().reset_kpis()
        self.kpi_provider.clear_weeding_kpis()

    def backup(self) -> dict:
        return {
            'with_drilling': self.with_drilling,
            'with_chopping': self.with_chopping,
            'chop_if_no_crops': self.chop_if_no_crops,
            'cultivated_crop': self.cultivated_crop,
        }

    def restore(self, data: dict[str, Any]) -> None:
        self.with_drilling = data.get('with_drilling', self.with_drilling)
        self.with_chopping = data.get('with_chopping', self.with_chopping)
        self.chop_if_no_crops = data.get('chop_if_no_crops', self.chop_if_no_crops)
        self.cultivated_crop = data.get('cultivated_crop', self.cultivated_crop)

    def clear(self) -> None:
        self.crops_to_handle = {}
        self.weeds_to_handle = {}

    def _update_time_and_distance(self):
        # TODO move this to base class?
        if self.state == 'idle':
            return
        if self.start_time is None:
            self.start_time = rosys.time()
        if self.last_pose is None:
            self.last_pose = self.system.odometer.prediction
            self.driven_distance = 0.0
        self.driven_distance += self.system.odometer.prediction.distance(self.last_pose)
        if self.driven_distance > 1:
            self.kpi_provider.increment_weeding_kpi('distance')
            self.driven_distance -= 1
        self.last_pose = self.system.odometer.prediction
        passed_time = rosys.time() - self.start_time
        if passed_time > 1:
            self.kpi_provider.increment_weeding_kpi('time')
            self.start_time = rosys.time()

    def settings_ui(self):
        super().settings_ui()
        ui.select(self.system.crop_category_names, label='cultivated crop', on_change=self.request_backup) \
            .bind_value(self, 'cultivated_crop').props('clearable') \
            .classes('w-40').tooltip('Set the cultivated crop which should be kept safe')
