import logging

import rosys

from .zedxmini_camera import ZedxminiCamera

SCAN_INTERVAL = 10


class ZedxminiCameraProvider(rosys.vision.CameraProvider[ZedxminiCamera], rosys.persistence.PersistentModule):

    def __init__(self) -> None:
        super().__init__()
        self.log = logging.getLogger('field_friend.zedxmini_camera_provider')

        rosys.on_shutdown(self.shutdown)
        rosys.on_repeat(self.update_device_list, SCAN_INTERVAL)
        rosys.on_startup(self.update_device_list)

    def backup(self) -> dict:
        for camera in self._cameras.values():
            self.log.info(f'backing up camera: {camera.name}')
        return super().backup()

    def restore(self, data: dict[str, dict]) -> None:
        for camera_data in data.get('cameras', {}).values():
            self.add_camera(ZedxminiCamera.from_dict(camera_data))

    async def update_device_list(self) -> None:
        if len(self._cameras) == 0:
            camera_information = await ZedxminiCamera.get_camera_information('localhost', 8003)
            if camera_information is None:
                return
            self.add_camera(ZedxminiCamera(id=str(camera_information['serial_number'])))
        camera = next(iter(self._cameras.values()))
        if camera.is_connected:
            return
        await camera.reconnect()

    async def shutdown(self) -> None:
        for camera in self._cameras.values():
            await camera.disconnect()

    @staticmethod
    def is_operable() -> bool:
        return True
