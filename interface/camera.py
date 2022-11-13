import rosys
from nicegui import ui


def camera(camera_provider: rosys.vision.CameraProvider):

    with ui.card().tight().classes('col').style('width: 600px') as card:
        if camera_provider.cameras.keys():
            _add_camera(card, list(camera_provider.cameras.values())[0], camera_provider)
        else:
            ui.image('assets/field_friend.webp').classes('w-full')
            ui.label('no camera available').classes('text-center')
            camera_provider.CAMERA_ADDED.register(lambda camera: _add_camera(card, camera, camera_provider))


def _add_camera(card, camera: rosys.vision.Camera, camera_provider: rosys.vision.CameraProvider):
    card.clear()
    with card:
        image_view = ui.interactive_image(camera_provider.get_latest_image_url(camera), cross=False) \
            .classes('w-full')

        async def update():
            await image_view.set_source(camera_provider.get_latest_image_url(camera))

        ui.timer(0.1, update)
        with ui.column().classes('col-grow'):
            info = ui.label().style('font-size:9pt')
            # ui.button(on_click=lambda: ui.open(calibration_page.path)).\
            #     props('icon=straighten size=sm').tooltip('Kalibrieren')
