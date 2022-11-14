import colorsys

import numpy as np
import rosys
from nicegui import ui
from nicegui.events import ValueChangeEventArguments

from .calibration_dialog import calibration_dialog


class camera:

    def __init__(self, camera_provider: rosys.vision.CameraProvider) -> None:
        self.camera_provider = camera_provider
        self.camera: rosys.vision.Camera = None
        self.image_view: ui.interactive_image = None
        self.calibration_dialog = calibration_dialog(camera_provider)
        with ui.card().tight().classes('col gap-4').style('width: 600px') as self.card:
            if camera_provider.cameras.keys():
                self.use_camera(list(camera_provider.cameras.values())[0])
            else:
                ui.image('assets/field_friend.webp').classes('w-full')
                ui.label('no camera available').classes('text-center')
                camera_provider.CAMERA_ADDED.register(self.use_camera)

    def use_camera(self, camera: rosys.vision.Camera) -> None:
        self.camera = camera
        self.card.clear()
        with self.card:
            self.image_view = ui.interactive_image(self.camera_provider.get_latest_image_url(camera), cross=False) \
                .classes('w-full')

            async def update():
                await self.image_view.set_source(self.camera_provider.get_latest_image_url(camera))

            ui.timer(0.1, update)
            with ui.row().classes('m-4 justify-end items-center'):
                self.show_mapping = ui.checkbox('Show Mapping', on_change=self.show_mapping)\
                    .tooltip('Show the mapping between camera and world coordinates')
                ui.button('calibrate', on_click=self.calibrate) \
                    .props('icon=straighten outline').tooltip('Calibrate camera')
            #ui.timer(2, lambda: self.calibrate, once=True)

    async def calibrate(self) -> None:
        result = await self.calibration_dialog.edit(self.camera)
        if result:
            self.show_mapping.value = True

    def show_mapping(self, args: ValueChangeEventArguments) -> None:
        if not args.value:
            self.image_view.svg_content = ''
            return

        grid_points = np.array([p.tuple for p in self.create_grid_points()])
        grid_image_points = self.camera.calibration.project_array_to_image(world_points=grid_points)
        c = len(grid_image_points)

        def get_color(i: int) -> str:
            rgb = colorsys.hsv_to_rgb(i/c, 1, 1)
            return f"#{int(rgb[0]*255):02x}{int(rgb[1]*255):02x}{int(rgb[2]*255):02x}"
        self.image_view.set_svg_content(''.join(f'<circle cx="{g[0]}" cy="{g[1]}" r="2" fill="{get_color(i)}"/>' for i,
                                                g in enumerate(grid_image_points)))

    @staticmethod
    def create_grid_points() -> list[rosys.geometry.Point3d]:
        result = []
        x = 0
        while x <= 0.7:
            y = -0.2
            while y <= 0.2:
                result.append(rosys.geometry.Point3d(x=x, y=y, z=0))
                y += 0.03
            x += 0.03
        return result
