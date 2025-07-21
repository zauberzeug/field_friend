from __future__ import annotations

import logging
from typing import TYPE_CHECKING

import rosys

if TYPE_CHECKING:
    from .system import System


class Capture:
    def __init__(self, system: System) -> None:
        self.log = logging.getLogger('field_friend.capture')
        self.inner_camera_provider = system.camera_provider
        self.inner_camera_detector = system.detector
        self.circle_sight_provider = system.mjpeg_camera_provider
        self.circle_sight_detector = system.circle_sight_detector
        self.circle_sight_positions = system.config.circle_sight_positions
        self.robot_id = system.robot_id

    async def front(self):
        await self.circle_sight(direction='front')

    async def circle_sight(self, *, direction: str | None = None):
        if self.circle_sight_provider is None:
            self.log.debug('No circle sight camera provider configured, skipping circle sight capture')
            return
        if not isinstance(self.circle_sight_detector, rosys.vision.DetectorHardware):
            self.log.debug('No DetectorHardware configured, skipping circle sight capture')
            return
        for camera_id, camera in self.circle_sight_provider.cameras.items():
            camera_name = self._id_to_camera_name(camera_id)
            if direction is not None and camera_name != direction:
                continue
            latest_image = camera.latest_captured_image
            if latest_image is None:
                self.log.debug(f'No image for camera {camera_id}')
                return
            tags = [] if camera_name is None else [camera_name]
            await self.circle_sight_detector.detect(latest_image, autoupload=rosys.vision.Autoupload.ALL, tags=tags, source=self.robot_id)
        rosys.notify(f'{direction} camera captured', type='positive')

    async def inner(self):
        if self.inner_camera_provider is None:
            self.log.debug('No camera provider configured, skipping inner camera capture')
            return
        if not isinstance(self.inner_camera_detector, rosys.vision.DetectorHardware):
            self.log.debug('No DetectorHardware configured, skipping inner camera capture')
            return
        camera = self.inner_camera_provider.first_connected_camera
        if camera is None:
            self.log.debug('No camera connected')
            return
        latest_image = camera.latest_captured_image
        if latest_image is None:
            self.log.debug('No image for main camera')
            return
        await self.inner_camera_detector.detect(latest_image, autoupload=rosys.vision.Autoupload.ALL, source=self.robot_id)
        rosys.notify('Main camera captured', type='positive')

    def _id_to_camera_name(self, camera_id: str) -> str | None:
        if self.circle_sight_positions is None:
            return None
        if camera_id.endswith(self.circle_sight_positions.right):
            return 'right'
        if camera_id.endswith(self.circle_sight_positions.back):
            return 'back'
        if camera_id.endswith(self.circle_sight_positions.front):
            return 'front'
        if camera_id.endswith(self.circle_sight_positions.left):
            return 'left'
        return None
