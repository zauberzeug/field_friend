import rosys
from nicegui import ui
from nicegui.events import MouseEventArguments


class calibration_dialog(ui.dialog):

    def __init__(self, camera_provider: rosys.vision.CameraProvider) -> None:
        super().__init__()
        self.camera_provider = camera_provider
        with self, ui.card().tight().style('max-width: 1000px'):
            ui.label('Calibration')
            self.calibration_image = ui.interactive_image('', on_mouse=self.on_image_clicked, cross=True)
            self.calibration_image.style('width: 1000px')

    def calibrate(self, camera: rosys.vision.Camera) -> None:
        update = self.calibration_image.set_source(self.camera_provider.get_latest_image_url(camera))
        rosys.task_logger.create_task(update)
        self.open()

    def on_image_clicked(self, e: MouseEventArguments) -> None:
        print(e)
