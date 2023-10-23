import logging

import rosys


class CameraSelector:
    FRONT_CAMERA_IDS = ['front_cam']
    BOTTOM_CAMERA_IDS = ['bottom_cam',
                         'usb-70090000.xusb-2.2',
                         'usb-70090000.xusb-2.2.1',
                         'usb-70090000.xusb-2.2.2',
                         'usb-70090000.xusb-2.2.3',
                         'usb-70090000.xusb-2.3',
                         'usb-70090000.xusb-2.3.1',
                         'usb-70090000.xusb-2.3.2',
                         'usb-70090000.xusb-2.3.3',
                         'usb-70090000.xusb-2.4',
                         'usb-70090000.xusb-2.4.1',
                         'usb-70090000.xusb-2.4.2',
                         'usb-70090000.xusb-2.4.3',
                         'usb-3610000.xhci-2.1'
                         'usb-3610000.xhci-2.2'
                         'usb-3610000.xhci-2.2.1'
                         'usb-3610000.xhci-2.2.2'
                         ]

    def __init__(self) -> None:
        self.CAMERA_SELECTED = rosys.event.Event()
        """A camera was selected"""

        self.camera_ids = {
            'front cam': self.FRONT_CAMERA_IDS,
            'bottom cam': self.BOTTOM_CAMERA_IDS,
        }
        self.cameras: dict[str, rosys.vision.Camera] = {}
        self.log = logging.getLogger('field_friend.camera_selector')

    async def use_camera(self, camera: rosys.vision.Camera) -> None:
        for camera_type, camera_ids_list in self.camera_ids.items():
            if camera.id in camera_ids_list:
                self.log.info(f'camera {camera.id} is a {camera_type}')
                self.cameras[camera_type] = camera
                await rosys.sleep(1)
                self.CAMERA_SELECTED.emit((camera_type, camera))
                return
        self.log.warning(f'camera {camera.id} is not a known camera')
