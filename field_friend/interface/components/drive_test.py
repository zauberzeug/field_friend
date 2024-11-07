import logging

import numpy as np
import rosys
from nicegui import events, ui
from rosys.automation import Automator
from rosys.driving import Driver, Odometer, PathSegment, Steerer, driver_object, keyboard_control
from rosys.geometry import Point, Pose, Spline
from rosys.pathplanning import path_object
from rosys.vision import CameraProvider

from ...hardware import FieldFriend
from .field_friend_object import field_friend_object


class DriveTest:

    def __init__(self, *,
                 field_friend: FieldFriend,
                 steerer: Steerer,
                 odometer: Odometer,
                 automator: Automator,
                 driver: Driver,
                 camera_provider: CameraProvider) -> None:
        self.field_friend = field_friend
        self.steerer = steerer
        self.odometer = odometer
        self.automator = automator
        self.driver = driver
        self.camera_provider = camera_provider
        self.log = logging.getLogger('field_friend.test')
        self.radius: float = 1.0
        self.right_turn: bool = True
        with ui.card().tight():
            def handle_click(event: events.SceneClickEventArguments) -> None:
                if event.click_type == 'dblclick':
                    position = self.odometer.prediction.point
                    scene.move_camera(x=position.x-0.5, y=position.y-1, z=1.5,
                                      look_at_x=position.x, look_at_y=position.y)
                    return
                if event.click_type == 'click':
                    scene.move_camera(x=event.hits[0].x-0.5, y=event.hits[0].y-1, z=1.5,
                                      look_at_x=event.hits[0].x, look_at_y=event.hits[0].y)
            with ui.scene(400, 300, on_click=handle_click) as scene:
                field_friend_object(self.odometer, self.camera_provider, self.field_friend)
                driver_object(self.driver)
                self.path3d = path_object()
                scene.move_camera(-0.5, -1, 1.5)
            with ui.row():
                key_controls = keyboard_control(self.steerer)
                with ui.column().classes('mt-4 m-4'):
                    with ui.row():
                        ui.number('speed', format='%.0f', max=9, min=1).props(
                            'dense outlined').classes('w-24').bind_value(key_controls, 'speed')
        with ui.card():
            with ui.row():
                ui.number('radius', value=1.0).bind_value_to(self, 'radius')
                ui.checkbox('right_turn', value=True).bind_value_to(self, 'right_turn')
                ui.button('drive radius', on_click=self.drive_radius)

                async def odometry_push() -> None:
                    time = rosys.time()
                    linear_target_speed = self.field_friend.wheels.linear_target_speed
                    # TODO: does 0 make sense if we have no velocity?
                    linear_speed = self.odometer.current_velocity.linear if self.odometer.current_velocity else 0
                    angular_target_speed = self.field_friend.wheels.angular_target_speed
                    angular_speed = self.odometer.current_velocity.angular if self.odometer.current_velocity else 0
                    linear_plot.push([time], [[linear_target_speed], [linear_speed]])
                    angular_plot.push([time], [[angular_target_speed], [angular_speed]])

                line_updates = ui.timer(0.1, odometry_push, active=False)
                ui.checkbox('active plots').bind_value(line_updates, 'active')

            with ui.row():
                with ui.column():
                    linear_plot = ui.line_plot(
                        n=2, update_every=10, figsize=(5, 4)).with_legend(
                        ['linear_target_speed', 'linear_speed'],
                        loc='upper left', ncol=2)
                angular_plot = ui.line_plot(
                    n=2, update_every=10, figsize=(5, 4)).with_legend(
                    ['angular_target_speed', 'angular_speed'],
                    loc='upper left', ncol=2)

    def drive_radius(self):
        current_pose = self.odometer.prediction
        if self.right_turn:
            target = self.odometer.prediction.transform(Point(x=self.radius, y=-self.radius))
        else:
            target = self.odometer.prediction.transform(Point(x=self.radius, y=self.radius))
        target_pose = Pose(x=target.x, y=target.y, yaw=current_pose.yaw - np.pi / 2
                           if self.right_turn else current_pose.yaw + np.pi / 2)
        spline = Spline.from_poses(current_pose, target_pose)
        path = [PathSegment(spline=spline)]
        self.path3d.update(path)
        self.automator.start(self.driver.drive_path(path))
