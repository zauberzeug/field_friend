import logging

import rosys
from rosys.vision.usb_camera.usb_camera_scanner import scan_for_connected_devices

from .zedxmini_camera import ZedxminiCamera

SCAN_INTERVAL = 10


class ZedxminiCameraProvider(rosys.vision.CameraProvider[ZedxminiCamera], rosys.persistence.PersistentModule):

    def __init__(self) -> None:
        super().__init__()
        self.log = logging.getLogger('field_friend.zedxmini_camera_provider')

        rosys.on_shutdown(self.shutdown)
        # rosys.on_repeat(self.update_device_list, SCAN_INTERVAL)
        rosys.on_startup(self.update_device_list)

    def backup(self) -> dict:
        # for camera in self._cameras.values():
        #     self.log.info(f'backing up camera: {camera.to_dict()}')
        # return {
        #     'cameras': {camera.id: camera.to_dict() for camera in self._cameras.values()}
        # }
        return {}

    def restore(self, data: dict[str, dict]) -> None:
        # for camera_data in data.get('cameras', {}).values():
        #     self.add_camera(ZedxminiCamera.from_dict(camera_data))
        pass

    @staticmethod
    async def scan_for_cameras() -> set[str]:
        return (await rosys.run.io_bound(scan_for_connected_devices)) or set()

    async def update_device_list(self) -> None:
        if len(self._cameras) == 0:
            # TODO: get camera id from api
            self.add_camera(ZedxminiCamera(camera_id='zedxmini-todo'))
        camera = list(self._cameras.values())[0]
        if camera.is_connected:
            return
        await camera.reconnect()

    async def shutdown(self) -> None:
        for camera in self._cameras.values():
            await camera.disconnect()

    @staticmethod
    def is_operable() -> bool:
        return True
