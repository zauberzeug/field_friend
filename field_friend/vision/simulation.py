import numpy as np
import rosys


def create_weedcam(usb_camera_provider: rosys.vision.UsbCameraProviderSimulation) \
        -> rosys.vision.UsbCamera:
    usb_camera_provider.cameras.clear()
    camera = usb_camera_provider.create_calibrated('front_cam', color='#555',
                                                   x=0, y=0, z=0.3,
                                                   roll=np.deg2rad(360-150), pitch=np.deg2rad(0), yaw=np.deg2rad(-90))
    usb_camera_provider.add_camera(camera)
    return camera
