import logging
import shutil
from typing import Any

import rosys
from rosys.vision.usb_camera.usb_camera_scanner import scan_for_connected_devices

from .calibratable_usb_camera import CalibratableUsbCamera

SCAN_INTERVAL = 10


class CalibratableUsbCameraProvider(rosys.vision.CameraProvider[CalibratableUsbCamera], rosys.persistence.Persistable):

    def __init__(self) -> None:
        super().__init__()
        self.log = logging.getLogger('field_friend.calibratable_usb_camera_provider')

        rosys.on_shutdown(self.shutdown)
        rosys.on_repeat(self.update_device_list, SCAN_INTERVAL)

    def backup_to_dict(self) -> dict[str, Any]:
        return super().backup_to_dict()

    # TODO: same as in zedxmini_camera_provider.py, refactor!
    def restore_from_dict(self, data: dict[str, dict]) -> None:
        for camera_data in data.get('cameras', {}).values():
            self.add_camera(CalibratableUsbCamera.from_dict(camera_data))

    @staticmethod
    async def scan_for_cameras() -> set[str]:
        return (await rosys.run.io_bound(scan_for_connected_devices)) or set()

    async def update_device_list(self) -> None:
        camera_uids = await self.scan_for_cameras()
        for uid in camera_uids:
            if uid not in self._cameras:
                self.add_camera(CalibratableUsbCamera(id=uid))
            await self._cameras[uid].connect()

    async def shutdown(self) -> None:
        for camera in self._cameras.values():
            await camera.disconnect()

    @staticmethod
    def is_operable() -> bool:
        return shutil.which('v4l2-ctl') is not None
