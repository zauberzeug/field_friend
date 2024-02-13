import logging
import shutil

import rosys
from rosys.vision.usb_camera.usb_camera_scanner import scan_for_connected_devices

from . import UsbCam

SCAN_INTERVAL = 10


class UsbCamProvider(rosys.vision.CameraProvider[UsbCam], rosys.persistence.PersistentModule):

    def __init__(self) -> None:
        super().__init__()

        self.log = logging.getLogger('field_friend.usb_cam_provider')

        rosys.on_shutdown(self.shutdown)
        rosys.on_repeat(self.update_device_list, SCAN_INTERVAL)

    def backup(self) -> dict:
        for camera in self._cameras.values():
            self.log.info(f'backing up camera: {camera.to_dict()}')
        return {
            'cameras': {camera.id: camera.to_dict() for camera in self._cameras.values()}
        }

    def restore(self, data: dict[str, dict]) -> None:
        for camera_data in data.get('cameras', {}).values():
            self.add_camera(UsbCam.from_dict(camera_data))

    @staticmethod
    async def scan_for_cameras() -> set[str]:
        return (await rosys.run.io_bound(scan_for_connected_devices)) or set()

    async def update_device_list(self) -> None:
        camera_uids = await self.scan_for_cameras()
        for uid in camera_uids:
            if uid not in self._cameras:
                self.add_camera(UsbCam(id=uid))
            await self._cameras[uid].connect()

    async def shutdown(self) -> None:
        for camera in self._cameras.values():
            await camera.disconnect()

    @staticmethod
    def is_operable() -> bool:
        return shutil.which('v4l2-ctl') is not None
