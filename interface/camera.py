import rosys
from nicegui import ui

from .calibration_dialog import calibration_dialog


class camera:

    def __init__(self, camera_provider: rosys.vision.CameraProvider) -> None:
        self.camera_provider = camera_provider
        self.calibration_dialog = calibration_dialog(camera_provider)
        with ui.card().tight().classes('col gap-4').style('width: 600px') as self.card:
            if camera_provider.cameras.keys():
                self.use_camera(list(camera_provider.cameras.values())[0])
            else:
                ui.image('assets/field_friend.webp').classes('w-full')
                ui.label('no camera available').classes('text-center')
                camera_provider.CAMERA_ADDED.register(self.use_camera)

    def use_camera(self, camera: rosys.vision.Camera):
        self.card.clear()
        with self.card:
            image_view = ui.interactive_image(self.camera_provider.get_latest_image_url(camera), cross=False) \
                .classes('w-full')

            async def update():
                await image_view.set_source(self.camera_provider.get_latest_image_url(camera))

            ui.timer(0.1, update)
            with ui.row().classes('m-4 justify-end'):
                ui.button('calibrate', on_click=lambda: self.calibration_dialog.edit(camera)) \
                    .props('icon=straighten outline').tooltip('Calibrate camera')
            ui.timer(2, lambda: self.calibration_dialog.edit(camera), once=True)
