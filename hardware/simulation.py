import rosys


def create_weedcam(usb_camera_provider: rosys.vision.UsbCameraProviderSimulation) \
        -> rosys.vision.UsbCamera:
    usb_camera_provider.cameras.clear()
    camera = usb_camera_provider.create('weedcam', color='#BBCEF2')
    usb_camera_provider.add_camera(camera)
    return camera
