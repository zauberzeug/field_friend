import logging
import rosys
from nicegui import events, ui

from .field_friend_object import field_friend_object
from .field_object import field_object
from .plant_object import plant_objects
from .visualizer_object import visualizer_object

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from field_friend.system import System


class robot_scene:

    def __init__(self, system: 'System'):
        self.log = logging.getLogger('field_friend.robot_scene')
        self.system = system
        self.scene_card = ui.card()
        self.scene_look = False
        self.locked_view = True

        with self.scene_card.tight().classes('w-full place-items-center').style('max-width: 100%; overflow: hidden;'):
            def toggle_lock():
                self.locked_view = not self.locked_view
                self.lock_view_button.props(
                    f'flat color={"primary" if self.locked_view else "grey"}')
            self.lock_view_button = ui.button(icon='sym_o_visibility_lock', on_click=toggle_lock).props('flat color=primary').style(
                'position: absolute; left: 1px; top: 1px; z-index: 500;').tooltip('Lock view to robot')

            with ui.scene(200, 200, on_click=self.handle_click, grid=True, on_drag_end=self.handle_drag).classes('w-full') as self.scene:
                field_friend_object(self.system.odometer, self.system.usb_camera_provider, self.system.field_friend)
                rosys.driving.driver_object(self.system.driver)
                plant_objects(self.system.plant_provider, self.system.big_weed_category_names +
                              self.system.small_weed_category_names)
                visualizer_object(self.system.automator, self.system.path_provider,
                                  self.system.mowing, self.system.weeding)
                field_object(self.system.field_provider)
                self.scene.move_camera(-0.5, -1, 2)

        ui.timer(rosys.config.ui_update_interval, self.update)

    def handle_drag(self, e: events.SceneDragEventArguments):
        for objects in self.scene.objects.values():
            if objects.name == e.object_name:
                objects.move(e.x, e.y, 0.02)
                break

        for plant in self.system.plant_provider.weeds + self.system.plant_provider.crops:
            check_id = f'plant_{plant.type}:{plant.id}'
            if check_id == e.object_name:
                plant.position.x = e.x
                plant.position.y = e.y
                break

    def handle_click(self, event: events.SceneClickEventArguments) -> None:
        if event.click_type == 'dblclick':
            self.scene_look = not self.scene_look
            if self.scene_look:
                height = 10.0
            else:
                height = 1.2
            self.scene.move_camera(z=height, duration=0.0)

    def update(self) -> None:
        if not self.locked_view:
            return
        position = self.system.odometer.prediction.point
        relative_camera_position = self.system.odometer.prediction.transform(rosys.geometry.Point(x=0.1, y=0.7))

        self.scene.move_camera(x=relative_camera_position.x, y=relative_camera_position.y,
                               look_at_x=position.x, look_at_y=position.y, duration=0.0)
