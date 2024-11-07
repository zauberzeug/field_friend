from __future__ import annotations

import logging
from typing import TYPE_CHECKING

import rosys
from nicegui import events, ui

from .field_friend_object import field_friend_object
from .field_object import FieldObject as field_object
from .plant_object import PlantObjects as plant_objects
from .visualizer_object import visualizer_object

if TYPE_CHECKING:
    from ...system import System


class RobotScene:

    def __init__(self, system: System) -> None:
        self.log = logging.getLogger('field_friend.robot_scene')
        self.system = system
        self.scene_card = ui.card()
        self.scene_look = False
        self.locked_view = True

        with self.scene_card.tight().classes('w-full place-items-center').style('max-width: 100%; overflow: hidden;'):
            def toggle_lock():
                self.locked_view = not self.locked_view
                self.lock_view_button.props(f'flat color={"primary" if self.locked_view else "grey"}')
            self.lock_view_button = ui.button(icon='sym_o_visibility_lock', on_click=toggle_lock).props('flat color=primary') \
                .style('position: absolute; left: 1px; top: 1px; z-index: 500;').tooltip('Lock view to robot')

            with ui.scene(200, 200, on_click=self.handle_click, grid=False).classes('w-full') as self.scene:
                field_friend_object(self.system.odometer, self.system.camera_provider, self.system.field_friend)
                rosys.driving.driver_object(self.system.driver)
                plant_objects(self.system)
                visualizer_object(self.system)
                field_object(self.system)
                self.scene.move_camera(-0.5, -1, 2)

        ui.timer(rosys.config.ui_update_interval, self.update)

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
