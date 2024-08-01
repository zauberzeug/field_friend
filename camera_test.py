from pyzed import sl

# Create a ZED camera object
zed = sl.Camera()
init_params = sl.InitParameters()
init_params.camera_resolution = sl.RESOLUTION.HD1080
init_params.camera_fps = 30

err = zed.open(init_params)
if err != sl.ERROR_CODE.SUCCESS:
    exit(1)
