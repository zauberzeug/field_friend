import rosys
from nicegui import ui


class robot_object(rosys.driving.robot_object):
    width = 0.63
    length = 0.78
    offset = 0.16
    chain_width = 0.145

    shape = rosys.geometry.Prism(
        outline=[
            (-offset, -width/2),
            (length - offset, -width/2),
            (length - offset, -width/2 + chain_width),
            (0, -width/2 + chain_width),
            (0, width/2 - chain_width),
            (length - offset, width/2 - chain_width),
            (length - offset, width/2),
            (-offset, width/2)
        ],
        height=0.40,
    )

    def __init__(self, odometer: rosys.driving.Odometer, camera_provider: rosys.vision.CameraProvider) -> None:
        super().__init__(self.shape, odometer, debug=True)
        ui.add_static_files('/assets', 'assets')
        self.with_stl('assets/field_friend.stl', x=-0.15, y=-0.3, z=0.05, scale=0.001, color='#6E93D6')
        with self:
            rosys.vision.camera_objects(camera_provider, rosys.vision.CameraProjector(camera_provider))
