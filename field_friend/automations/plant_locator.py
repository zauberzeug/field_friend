from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Any

import rosys
from nicegui import ui
from rosys.vision import Autoupload, DetectorSimulation
from rosys.vision.detections import Category
from rosys.vision.detector import DetectorException, DetectorInfo

from ..vision.detector_hardware import DetectorHardware
from ..vision.zedxmini_camera import StereoCamera
from .entity_locator import EntityLocator
from .plant import Plant

WEED_CATEGORY_NAME = ['weed', 'weedy_area', 'coin', 'big_weed']
CROP_CATEGORY_NAME: dict[str, str] = {}
MINIMUM_CROP_CONFIDENCE = 0.3
MINIMUM_WEED_CONFIDENCE = 0.3


if TYPE_CHECKING:
    from ..system import System


class DetectorError(Exception):
    pass


class PlantLocator(EntityLocator):
    def __init__(self, system: System) -> None:
        super().__init__(system)
        self.log = logging.getLogger('field_friend.plant_locator')
        self.camera_provider = system.camera_provider
        self.detector = system.detector
        self.plant_provider = system.plant_provider
        self.robot_locator = system.robot_locator
        self.robot_id = system.robot_id
        self.detector_info: DetectorInfo | None = None
        self.tags: list[str] = []
        self.is_paused = True
        self.autoupload: Autoupload = Autoupload.DISABLED
        self.upload_images: bool = False
        self.weed_category_names: list[str] = WEED_CATEGORY_NAME
        self.crop_category_names: dict[str, str] = CROP_CATEGORY_NAME
        self.minimum_crop_confidence: float = MINIMUM_CROP_CONFIDENCE
        self.minimum_weed_confidence: float = MINIMUM_WEED_CONFIDENCE
        if isinstance(self.detector, DetectorHardware):
            rosys.on_repeat(lambda: self.detector.set_outbox_mode(value=self.upload_images), 1.0)
        if system.is_real:
            self.teltonika_router = system.teltonika_router
            self.teltonika_router.CONNECTION_CHANGED.register(self.set_upload_images)
            self.teltonika_router.MOBILE_UPLOAD_PERMISSION_CHANGED.register(self.set_upload_images)
        self.detector_error = False
        self.last_detection_time = rosys.time()
        if self.camera_provider is None:
            self.log.warning('no camera provider configured, cant locate plants')
            return
        assert self.detector is not None
        self.detector.NEW_DETECTIONS.register(lambda e: setattr(self, 'last_detection_time', rosys.time()))
        rosys.on_repeat(self._detect_plants, 0.01)  # as fast as possible, function will sleep if necessary
        rosys.on_repeat(self._detection_watchdog, 0.5)
        rosys.on_startup(self.fetch_detector_info)

    def backup_to_dict(self) -> dict[str, Any]:
        self.log.debug(f'backup: autoupload: {self.autoupload}')
        return super().backup_to_dict() | {
            'minimum_weed_confidence': self.minimum_weed_confidence,
            'minimum_crop_confidence': self.minimum_crop_confidence,
            'autoupload': self.autoupload.value,
            'upload_images': self.upload_images,
            'tags': self.tags,
        }

    def restore_from_dict(self, data: dict[str, Any]) -> None:
        super().restore_from_dict(data)
        self.minimum_weed_confidence = data.get('minimum_weed_confidence', MINIMUM_WEED_CONFIDENCE)
        self.minimum_crop_confidence = data.get('minimum_crop_confidence', MINIMUM_CROP_CONFIDENCE)
        self.autoupload = Autoupload(data.get('autoupload', self.autoupload)) \
            if 'autoupload' in data else Autoupload.DISABLED
        self.log.debug(f'self.autoupload: {self.autoupload}')
        self.upload_images = data.get('upload_images', self.upload_images)
        self.tags = data.get('tags', self.tags)

    async def _detect_plants(self) -> None:
        if self.is_paused:
            await rosys.sleep(0.01)
            return
        t = rosys.time()
        assert self.camera_provider is not None
        camera = next((camera for camera in self.camera_provider.cameras.values() if camera.is_connected), None)
        if not camera:
            self.log.error('no connected camera found')
            return
        assert isinstance(camera, rosys.vision.CalibratableCamera)
        if camera.calibration is None:
            self.log.error(f'no calibration found for camera {camera.name}')
            raise DetectorError()
        if not self.crop_category_names:
            self.log.warning('No crop categories defined')
            await self.fetch_detector_info()
        new_image = camera.latest_captured_image
        if new_image is None or new_image.detections:
            await rosys.sleep(0.01)
            return
        assert self.detector is not None
        await self.detector.detect(new_image, autoupload=self.autoupload, tags=[*self.tags, self.robot_id, 'autoupload'], source=self.robot_id)
        if rosys.time() - t < 0.01:  # ensure maximum of 100 Hz
            await rosys.sleep(0.01 - (rosys.time() - t))
        if not new_image.detections:
            return

        for d in new_image.detections.points:
            if isinstance(self.detector, rosys.vision.DetectorSimulation):
                # NOTE we drop detections at the edge of the vision because in reality they are blocked by the chassis
                dead_zone = 80
                if d.cx < dead_zone or d.cx > new_image.size.width - dead_zone or d.cy < dead_zone:
                    continue
            image_point = rosys.geometry.Point(x=d.cx, y=d.cy)
            world_point_3d: rosys.geometry.Point3d | None = None
            if isinstance(camera, StereoCamera):
                world_point_3d = camera.calibration.project_from_image(image_point)
                # TODO: 3d detection
                # camera_point_3d: Point3d | None = await camera.get_point(
                #     int(d.cx), int(d.cy))
                # if camera_point_3d is None:
                #     self.log.error('could not get a depth value for detection')
                #     continue
                # camera.calibration.extrinsics = camera.calibration.extrinsics.as_frame(
                #     'zedxmini').in_frame(self.robot_locator.pose_frame)
                # world_point_3d = camera_point_3d.in_frame(camera.calibration.extrinsics).resolve()
            else:
                world_point_3d = camera.calibration.project_from_image(image_point)
            if world_point_3d is None:
                self.log.error('Failed to generate world point from %s', image_point)
                continue
            plant = Plant(type=d.category_name,
                          detection_time=rosys.time(),
                          detection_image=new_image)
            plant.positions.append(world_point_3d)
            plant.confidences.append(d.confidence)
            if d.category_name in self.weed_category_names and d.confidence >= self.minimum_weed_confidence:
                await self.plant_provider.add_weed(plant)
            elif d.category_name in self.crop_category_names and d.confidence >= self.minimum_crop_confidence:
                self.plant_provider.add_crop(plant)
            elif d.category_name not in self.crop_category_names and d.category_name not in self.weed_category_names:
                self.log.error('Detected category "%s" is unknown', d.category_name)

    def _detection_watchdog(self) -> None:
        if self.is_paused:
            return
        if rosys.time() - self.last_detection_time > 1.0 and not self.detector_error:
            self.log.debug('No new detections')
            self.detector_error = True
            return
        if rosys.time() - self.last_detection_time <= 1.0 and self.detector_error:
            self.detector_error = False
            self.log.debug('Detection error resolved')

    def developer_ui(self) -> None:
        ui.label('Plant Locator').classes('text-center text-bold')
        super().developer_ui()
        with ui.column().classes('w-64'):
            with ui.row():
                ui.number('Min. crop confidence', format='%.2f', step=0.05, min=0.0, max=1.0, on_change=self.request_backup) \
                    .props('dense outlined') \
                    .classes('w-28') \
                    .bind_value(self, 'minimum_crop_confidence') \
                    .tooltip(f'Set the minimum crop confidence for the detection (default: {MINIMUM_CROP_CONFIDENCE:.2f})')
                ui.number('Min. weed confidence', format='%.2f', step=0.05, min=0.0, max=1.0, on_change=self.request_backup) \
                    .props('dense outlined') \
                    .classes('w-28') \
                    .bind_value(self, 'minimum_weed_confidence') \
                    .tooltip(f'Set the minimum weed confidence for the detection(default: {MINIMUM_WEED_CONFIDENCE: .2f})')
                options = list(rosys.vision.Autoupload)
                ui.select(options, label='Autoupload', on_change=self.request_backup) \
                    .props('dense outlined') \
                    .classes('w-28') \
                    .bind_value(self, 'autoupload') \
                    .tooltip('Set the autoupload for the weeding automation')
                ui.button('Fetch detector info', on_click=self.fetch_detector_info)
            ui.label().bind_text_from(self, 'detector_info',
                                      backward=lambda info: f'Detector version: {info.current_version}/{info.target_version}' if info else 'Detector version: unknown')

            @ui.refreshable
            def chips():
                with ui.row().classes('gap-0'):
                    ui.chip(self.robot_id).props('outline')

                    def update_tags(tag_to_remove: str) -> None:
                        self.tags.remove(tag_to_remove)
                        chips.refresh()
                        self.request_backup()
                        self.log.debug(f'tags: {self.tags}')
                    for tag in self.tags:
                        ui.chip(tag, removable=True).props('outline') \
                            .on('remove', lambda t=tag: update_tags(t))

            def add_chip():
                self.tags.append(label_input.value)
                self.request_backup()
                chips.refresh()
                label_input.value = ''

            with ui.row().classes('items-center'):
                label_input = ui.input().on('keydown.enter', add_chip).classes('w-24').props('dense') \
                    .tooltip('Add a tag for the Learning Loop')
                with label_input.add_slot('append'):
                    ui.button(icon='add', on_click=add_chip).props('round dense flat')
            with ui.row().classes('items-center'):
                chips()

    def set_upload_images(self):
        if self.teltonika_router.mobile_upload_permission:
            self.upload_images = True
        elif self.teltonika_router.current_connection in ['wifi', 'ether']:
            self.upload_images = True
        else:
            self.upload_images = False

    async def fetch_detector_info(self) -> bool:
        assert self.detector is not None
        try:
            detector_info: DetectorInfo = await self.detector.fetch_detector_info()
        except DetectorException as e:
            self.log.error(f'Could not fetch detector info: {e}')
            return False
        if isinstance(self.detector, DetectorSimulation):
            detector_info.categories = [Category(uuid=name, name=name) for name in ['coin_with_hole', 'borrietsch', 'estragon', 'feldsalat', 'garlic', 'jasione', 'kohlrabi', 'liebstoeckel', 'maize', 'minze', 'onion',
                                        'oregano_majoran', 'pastinake', 'petersilie', 'pimpinelle', 'red_beet', 'salatkopf', 'schnittlauch', 'sugar_beet', 'thymian_bohnenkraut', 'zitronenmelisse', ]]
            detector_info.current_version = 'simulation'
            detector_info.target_version = 'simulation'
        filtered_crops: list[str] = [
            category.name for category in detector_info.categories if category.name not in WEED_CATEGORY_NAME]
        self.crop_category_names.update({crop_name: crop_name.replace('_', ' ').title()
                                        for crop_name in filtered_crops})
        self.detector_info = detector_info
        self.log.debug('Fetched detector info: %s and crops: %s', detector_info, self.crop_category_names)
        return True
