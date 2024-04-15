import asyncio
import logging

import rosys


class DetectorError(Exception):
    pass


class PersonLocator:

    def __init__(self,
                 mjpeg_camera_provider: rosys.vision.MjpegCameraProvider,
                 detector: rosys.vision.Detector,
                 ) -> None:
        self.log = logging.getLogger('field_friend.person_detection')
        self.camera_provider = mjpeg_camera_provider
        self.detector = detector
        self.is_paused = True
        rosys.on_repeat(self._detect_persons, 0.01)  # as fast as possible, function will sleep if necessary

    async def _detect_persons(self) -> None:
        if self.is_paused:
            await asyncio.sleep(0.01)
            return
        t = rosys.time()
        for camera in self.camera_provider.cameras.values():
            new_image = camera.latest_captured_image
            if new_image is None or new_image.detections:
                await asyncio.sleep(0.01)
                continue
            await self.detector.detect(new_image)
            if rosys.time() - t < 0.01:  # ensure maximum of 100 Hz
                await asyncio.sleep(0.01 - (rosys.time() - t))
            if not new_image.detections:
                continue

    def pause(self) -> None:
        self.log.info('pausing plant detection')
        self.is_paused = True

    def resume(self) -> None:
        self.log.info('resuming plant detection')
        self.is_paused = False
