import logging
from typing import TYPE_CHECKING, Any

import aiohttp
import rosys
from nicegui import ui
from rosys.vision import Autoupload

from ..vision.zedxmini_camera import StereoCamera
from .plant import Plant

WEED_CATEGORY_NAME = ['coin', 'weed', 'weedy_area', ]
CROP_CATEGORY_NAME: dict[str, str] = {}
MINIMUM_CROP_CONFIDENCE = 0.3
MINIMUM_WEED_CONFIDENCE = 0.3


if TYPE_CHECKING:
    from ..system import System


class DetectorError(Exception):
    pass


class PlantLocator(rosys.persistence.PersistentModule):

    def __init__(self, system: 'System') -> None:
        super().__init__()
        self.log = logging.getLogger('field_friend.plant_detection')
        self.camera_provider = system.camera_provider
        self.detector = system.detector
        self.plant_provider = system.plant_provider
        self.robot_locator = system.robot_locator
        self.robot_name = system.version
        self.tags: list[str] = []
        self.is_paused = True
        self.autoupload: Autoupload = Autoupload.DISABLED
        self.upload_images: bool = False
        self.weed_category_names: list[str] = WEED_CATEGORY_NAME
        self.crop_category_names: dict[str, str] = CROP_CATEGORY_NAME
        self.minimum_crop_confidence: float = MINIMUM_CROP_CONFIDENCE
        self.minimum_weed_confidence: float = MINIMUM_WEED_CONFIDENCE
        rosys.on_repeat(self._detect_plants, 0.01)  # as fast as possible, function will sleep if necessary
        if isinstance(self.detector, rosys.vision.DetectorHardware):
            port = self.detector.port
            rosys.on_repeat(lambda: self.set_outbox_mode(value=self.upload_images, port=port), 1.0)
        if system.is_real:
            self.teltonika_router = system.teltonika_router
            self.teltonika_router.CONNECTION_CHANGED.register(self.set_upload_images)
            self.teltonika_router.MOBILE_UPLOAD_PERMISSION_CHANGED.register(self.set_upload_images)
        self.detector_error = False
        self.last_detection_time = rosys.time()
        self.detector.NEW_DETECTIONS.register(lambda e: setattr(self, 'last_detection_time', rosys.time()))
        rosys.on_repeat(self._detection_watchdog, 0.5)
        rosys.on_startup(self.get_crop_names)

    def backup(self) -> dict:
        self.log.debug(f'backup: autoupload: {self.autoupload}')
        return {
            'minimum_weed_confidence': self.minimum_weed_confidence,
            'minimum_crop_confidence': self.minimum_crop_confidence,
            'autoupload': self.autoupload.value,
            'upload_images': self.upload_images,
            'tags': self.tags,
        }

    def restore(self, data: dict[str, Any]) -> None:
        self.minimum_weed_confidence = data.get('minimum_weed_confidence', self.minimum_weed_confidence)
        self.minimum_crop_confidence = data.get('minimum_crop_confidence', self.minimum_crop_confidence)
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
        camera = next((camera for camera in self.camera_provider.cameras.values() if camera.is_connected), None)
        if not camera:
            self.log.error('no connected camera found')
            return
        assert isinstance(camera, rosys.vision.CalibratableCamera)
        if camera.calibration is None:
            self.log.error(f'no calibration found for camera {camera.name}')
            raise DetectorError()
        new_image = camera.latest_captured_image
        if new_image is None or new_image.detections:
            await rosys.sleep(0.01)
            return
        await self.detector.detect(new_image, autoupload=self.autoupload, tags=[*self.tags, self.robot_name, 'autoupload'])
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
                self.log.error('could not generate world point of detection, calibration error')
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
                self.log.info(f'{d.category_name} not in categories')

    def pause(self) -> None:
        if self.is_paused:
            return
        self.log.debug('pausing plant detection')
        self.is_paused = True

    def resume(self) -> None:
        if not self.is_paused:
            return
        self.last_detection_time = rosys.time()
        self.log.debug('resuming plant detection')
        self.is_paused = False

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

    async def get_outbox_mode(self, port: int) -> bool | None:
        # TODO: not needed right now, but can be used when this code is moved to the DetectorHardware code
        # TODO: active cleaner already has implemented this
        url = f'http://localhost:{port}/outbox_mode'
        async with aiohttp.request('GET', url) as response:
            if response.status != 200:
                self.log.error(f'Could not get outbox mode on port {port} - status code: {response.status}')
                return None
            response_text = await response.text()
        return response_text == 'continuous_upload'

    async def set_outbox_mode(self, value: bool, port: int) -> None:
        url = f'http://localhost:{port}/outbox_mode'
        async with aiohttp.request('PUT', url, data='continuous_upload' if value else 'stopped') as response:
            if response.status != 200:
                self.log.error(f'Could not set outbox mode to {value} on port {port} - status code: {response.status}')
                return
            self.log.debug(f'Outbox_mode was set to {value} on port {port}')

    def settings_ui(self) -> None:
        ui.number('Min. crop confidence', format='%.2f', step=0.05, min=0.0, max=1.0, on_change=self.request_backup) \
            .props('dense outlined') \
            .classes('w-24') \
            .bind_value(self, 'minimum_crop_confidence') \
            .tooltip(f'Set the minimum crop confidence for the detection (default: {MINIMUM_CROP_CONFIDENCE:.2f})')
        ui.number('Min. weed confidence', format='%.2f', step=0.05, min=0.0, max=1.0, on_change=self.request_backup) \
            .props('dense outlined') \
            .classes('w-24') \
            .bind_value(self, 'minimum_weed_confidence') \
            .tooltip(f'Set the minimum weed confidence for the detection  (default: {MINIMUM_WEED_CONFIDENCE:.2f})')
        options = list(rosys.vision.Autoupload)
        ui.select(options, label='Autoupload', on_change=self.request_backup) \
            .bind_value(self, 'autoupload') \
            .classes('w-24').tooltip('Set the autoupload for the weeding automation')

        @ui.refreshable
        def chips():
            with ui.row().classes('gap-0'):
                ui.chip(self.robot_name).props('outline')

                def update_tags(tag_to_remove: str) -> None:
                    self.tags.remove(tag_to_remove)
                    chips.refresh()
                    self.request_backup()
                    self.log.info(f'tags: {self.tags}')
                for tag in self.tags:
                    ui.chip(tag, removable=True).props('outline') \
                        .on('remove', lambda t=tag: update_tags(t))

        def add_chip():
            self.tags.append(label_input.value)
            self.request_backup()
            chips.refresh()
            label_input.value = ''

        with ui.row().classes('items-center'):
            chips()
            label_input = ui.input().on('keydown.enter', add_chip).classes('w-24').props('dense') \
                .tooltip('Add a tag for the Learning Loop')
            with label_input.add_slot('append'):
                ui.button(icon='add', on_click=add_chip).props('round dense flat')

    def set_upload_images(self):
        if self.teltonika_router.mobile_upload_permission:
            self.upload_images = True
        elif self.teltonika_router.current_connection in ['wifi', 'ether']:
            self.upload_images = True
        else:
            self.upload_images = False

    async def get_crop_names(self) -> dict[str, str]:
        if isinstance(self.detector, rosys.vision.DetectorSimulation):
            simulated_crop_names: list[str] = ['coin_with_hole', 'borrietsch', 'estragon', 'feldsalat', 'garlic', 'jasione', 'kohlrabi', 'liebstoeckel', 'maize', 'minze', 'onion',
                                               'oregano_majoran', 'pastinake', 'petersilie', 'pimpinelle', 'red_beet', 'salatkopf', 'schnittlauch', 'sugar_beet', 'thymian_bohnenkraut', 'zitronenmelisse', ]

            CROP_CATEGORY_NAME.update({name: name.replace('_', ' ').title() for name in simulated_crop_names})
            self.crop_category_names = CROP_CATEGORY_NAME
            return CROP_CATEGORY_NAME
        port = self.detector.port
        url = f'http://localhost:{port}/about'
        async with aiohttp.request('GET', url) as response:
            if response.status != 200:
                self.log.error(f'Could not get crop names on port {port} - status code: {response.status}')
                return {}
            response_text = await response.json()
        crop_names: list[str] = [category['name'] for category in response_text['model_info']['categories']]
        weeds = ['weed', 'weedy_area', 'coin', 'danger', 'big_weed']
        for weed in weeds:
            crop_names.remove(weed)
        CROP_CATEGORY_NAME.update({name: name.replace('_', ' ').title() for name in crop_names})
        self.crop_category_names = CROP_CATEGORY_NAME
        return CROP_CATEGORY_NAME
