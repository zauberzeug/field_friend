# pylint: disable=attribute-defined-outside-init
# pylint: disable=duplicate-code
# TODO: refactor this and monitoring.py
from __future__ import annotations

import colorsys
import logging
from datetime import datetime
from typing import TYPE_CHECKING

import numpy as np
import rosys
from nicegui import ui
from nicegui.events import MouseEventArguments
from rosys.geometry import Point, Point3d, Pose3d
from rosys.vision import CalibratableCamera

from ...automations.implements.tornado import Tornado as TornadoImplement
from ...automations.implements.weeding_implement import WeedingImplement
from ...automations.plant_locator import Plant
from ...hardware import Axis, FlashlightPWM, FlashlightPWMV2, Sprayer
from ...vision.zedxmini_camera import StereoCamera
from .calibration_dialog import CalibrationDialog as calibration_dialog

if TYPE_CHECKING:
    from ...system import System


class CameraCard:
    MAX_DETECTION_AGE = 2.0

    def __init__(self, system: System, *,
                 shrink_factor: float = 3.0,
                 show_plants: bool = True,
                 show_detections: bool = False,
                 show_plants_to_handle: bool = False) -> None:
        self.log = logging.getLogger('field_friend.camera_card')
        self.system = system
        self.automator = system.automator
        self.camera_provider = system.camera_provider
        self.detector = system.detector
        self.field_friend = system.field_friend
        self.robot_locator = system.robot_locator
        self.plant_locator = system.plant_locator
        self.plant_provider = system.plant_provider
        self.puncher = system.puncher
        self.shrink_factor: float = shrink_factor

        self.show_plants: bool = show_plants
        self.show_detections: bool = show_detections
        self.show_plants_to_handle: bool = show_plants_to_handle
        self.show_mapping: bool = False

        self.camera: CalibratableCamera | None = None
        self.image_view: ui.interactive_image | None = None
        self.mouse_over_context: ui.context_menu | None = None
        assert self.camera_provider is not None
        self.calibration_dialog = calibration_dialog(self.camera_provider, self.robot_locator)
        self.camera_card = ui.card()
        self.flashlight_toggled: bool = False
        with self.camera_card.tight().classes('w-full'):
            ui.label('no camera available').classes('text-center')
            ui.image('assets/field_friend.webp').classes('w-full')
        ui.timer(0.2 if system.is_real else 0.05, self.update_content)

    # TODO: refactor this and use refreshable, remove pylint ignore at top
    def use_camera(self, cam: CalibratableCamera) -> None:
        self.camera = cam
        self.camera_card.clear()
        with self.camera_card:
            if self.field_friend.flashlight and isinstance(self.field_friend.flashlight, FlashlightPWM | FlashlightPWMV2):
                self.flashlight_toggled = False

                async def toggle_flashlight():
                    # TODO: move into flashlight class
                    if not self.field_friend.flashlight:
                        rosys.notify('No flashlight found')
                        return
                    self.flashlight_toggled = not self.flashlight_toggled
                    flashlight_button.props(f'flat color={"grey" if not self.flashlight_toggled else "primary"} '
                                            f'icon={"flashlight_off" if not self.flashlight_toggled else "flashlight_on"}')
                    if self.flashlight_toggled:
                        await self.field_friend.flashlight.turn_on()
                        rosys.notify('Flashlight turned on')
                    else:
                        await self.field_friend.flashlight.turn_off()
                        rosys.notify('Flashlight turned off')
                flashlight_button = ui.button(icon='flashlight_off', on_click=toggle_flashlight) \
                    .props('flat color=grey').style('position: absolute; left: 1px; top: 1px; z-index: 500;') \
                    .bind_enabled_from(self.automator, 'is_running', backward=lambda x: not x)

            with ui.button(icon='menu').props('flat color=primary') \
                    .style('position: absolute; right: 1px; top: 1px; z-index: 500;'):
                with ui.menu():
                    with ui.menu_item():
                        ui.checkbox('Detecting Plants').bind_value(self.plant_locator, 'is_paused',
                                                                   backward=lambda x: not x, forward=lambda x: not x) \
                            .tooltip('Pause plant locator').bind_enabled_from(self.automator, 'is_running', backward=lambda x: not x)
                    with ui.menu_item():
                        ui.checkbox('Show plants').bind_value(self, 'show_plants') \
                            .tooltip('Show the plants in the image')
                    with ui.menu_item():
                        ui.checkbox('Show detections').bind_value(self, 'show_detections') \
                            .tooltip('Show the detections in the image')
                    with ui.menu_item():
                        ui.checkbox('Show weeding order').bind_value_to(self, 'show_plants_to_handle') \
                            .tooltip('Show the order of the weeding implement targets')
                    ui.separator()
                    with ui.menu_item():
                        ui.checkbox('Mapping').bind_value(self, 'show_mapping') \
                            .tooltip('Show the mapping between camera and world coordinates')
                    with ui.menu_item():
                        ui.button('calibrate', on_click=self.calibrate) \
                            .props('icon=straighten outline').tooltip('Calibrate camera')

            events = ['mousemove', 'mouseout', 'mouseup', 'contextmenu']
            self.image_view = ui.interactive_image('', cross=True, on_mouse=self._on_mouse_move, events=events) \
                .classes('w-full')
            with self.image_view:
                self.mouse_over_context = ui.context_menu()
            with ui.row():
                self.debug_position = ui.label('\u200b')

    def _on_mouse_move(self, e: MouseEventArguments):
        if self.camera is None:
            return
        if not isinstance(self.camera, CalibratableCamera):
            self.log.debug('Camera type not calibratable: %s', self.camera)
            return
        point2d = Point(x=e.image_x * self.shrink_factor, y=e.image_y * self.shrink_factor)
        if self.camera.calibration is None:
            self.debug_position.set_text(f'{point2d} no calibration')
            return
        point3d: Point3d | None = None
        if isinstance(self.camera, StereoCamera):
            # TODO: too many calls
            # camera_point_3d: Point3d | None = self.camera.get_point(
            #     int(point2d.x), int(point2d.y))
            # if camera_point_3d is None:
            #     return
            # camera_point_3d = camera_point_3d.in_frame(self.robot_locator.pose_frame)
            # world_point_3d = camera_point_3d.resolve()
            # self.log.info(f'camera_point_3d: {camera_point_3d} -> world_point_3d: {world_point_3d.tuple}')
            pass
        else:
            point3d = self.camera.calibration.project_from_image(point2d)

        if e.type == 'mousemove' and point3d is not None:
            point3d_in_locator_frame = point3d.relative_to(self.robot_locator.pose_frame)
            self.debug_position.set_text(f'{point2d} -> {point3d_in_locator_frame}')
        if e.type == 'mouseup':
            if self.camera.calibration is None:
                return
        if e.type == 'mouseout':
            self.debug_position.set_text('\u200b')

        if e.type == 'contextmenu' and point3d is not None:
            self._plant_context_menu(point3d)

    def _plant_context_menu(self, point3d: Point3d) -> None:
        def format_time(timestamp: float) -> str:
            dt = datetime.fromtimestamp(timestamp)
            return dt.strftime('%H:%M:%S') + f':{int(dt.microsecond/1000):03d}'
        if self.mouse_over_context is None:
            return
        self.mouse_over_context.clear()
        plants = sorted(self.plant_provider.get_relevant_crops(point3d, max_distance=0.02, min_confidence=0.0) +
                        self.plant_provider.get_relevant_weeds(point3d, max_distance=0.02, min_confidence=0.0),
                        key=lambda plant: plant.position.distance(point3d))
        if not plants:
            self.mouse_over_context.close()
            return
        plant = plants[0]
        with self.mouse_over_context.classes('p-2'):
            ui.label(f'{plant.id}').classes('text-subtitle2')
            with ui.grid(columns=2).classes('gap-0'):
                ui.label('Type:').classes('text-subtitle2')
                is_crop = plant.type in self.plant_locator.crop_category_names
                ui.label(f'{plant.type}').style(f'color: {"green" if is_crop else "red"}') \
                    .tooltip(f'The plant is a {"crop" if is_crop else "weed"}')
                ui.label('Position:').classes('text-subtitle2')
                position2d = plant.position.projection()
                x_std = np.std([p.x for p in plant.positions], dtype=np.float64)
                y_std = np.std([p.y for p in plant.positions], dtype=np.float64)
                with ui.column().classes('gap-0'):
                    ui.label(f'x: {position2d.x:.3f}m ± {x_std:.3f}')
                    ui.label(f'y: {position2d.y:.3f}m ± {y_std:.3f}')
                ui.label('Relative position:').classes('text-subtitle2')
                relative_position = plant.position.relative_to(self.system.robot_locator.pose_frame)
                with ui.column().classes('gap-0'):
                    ui.label(f'x: {relative_position.x:.3f}m')
                    ui.label(f'y: {relative_position.y:.3f}m')
                ui.label('Detection time:').classes('text-subtitle2')
                ui.label(f'{format_time(plant.detection_time)}')
                ui.label('Detections:').classes('text-subtitle2')
                ui.label(f'{len(plant.confidences)}')
                ui.label('Confidence:').classes('text-subtitle2')
                confidence = plant.confidence
                combined_threshold = self.plant_provider.minimum_combined_crop_confidence if is_crop else \
                    self.plant_provider.minimum_combined_weed_confidence
                above_threshold = confidence > combined_threshold
                ui.label(f'{confidence:.2f}').style(f'color: {"green" if above_threshold else "red"}') \
                    .tooltip(f'The confidence is {"above" if above_threshold else "below"} the threshold of {combined_threshold}')
                ui.label('Latest confidence:').classes('text-subtitle2')
                mean_confidence = float(np.mean(plant.confidences))
                std_confidence = float(np.std(plant.confidences))
                threshold = self.plant_locator.minimum_crop_confidence if is_crop else self.plant_locator.minimum_weed_confidence
                above_threshold = mean_confidence > threshold
                ui.label(f'{mean_confidence:.2f} ± {std_confidence:.2f}') \
                    .style(f'color: {"green" if above_threshold else "red"}') \
                    .tooltip(f'The confidence is {"above" if above_threshold else "below"} the threshold of {threshold}')

    async def calibrate(self) -> None:
        assert self.camera_provider is not None
        assert self.camera is not None
        result = await self.calibration_dialog.edit(self.camera)
        if result:
            self.show_mapping = True
            self.camera_provider.request_backup()

    def update_content(self) -> None:
        assert self.camera_provider is not None
        cameras = list(self.camera_provider.cameras.values())
        active_camera = next((camera for camera in cameras if camera.is_connected), None)
        if active_camera is None:
            if self.camera:
                self.camera = None
                self.camera_card.clear()
                with self.camera_card:
                    ui.label('no camera available').classes('text-center')
                    ui.image('assets/field_friend.webp').classes('w-full')
            return
        if (self.camera is None or self.camera != active_camera) and isinstance(active_camera, CalibratableCamera):
            self.use_camera(active_camera)
        if self.shrink_factor > 1:
            url = f'{active_camera.get_latest_image_url()}?shrink={self.shrink_factor}'
        else:
            url = active_camera.get_latest_image_url()
        if self.image_view is None:
            return
        self.image_view.set_source(url)
        if self.camera is None or self.camera.calibration is None:
            return
        image = active_camera.latest_detected_image
        svg = ''
        if self.show_mapping:
            svg += self.build_svg_for_mapping()
        if self.show_plants:
            svg += self.build_svg_for_plant_provider()
        image_age = rosys.time() - image.time if image else 0
        if self.show_detections and image and image.detections and image_age < self.MAX_DETECTION_AGE:
            svg += self.detections_to_svg(image.detections)

        if isinstance(self.system.current_implement, WeedingImplement) and self.field_friend.y_axis is not None:
            svg += self.build_svg_for_work_area()
            svg += self.build_svg_for_tool_position()
            svg += self.build_svg_for_tool_axis()
            if self.show_plants_to_handle:
                svg += self.build_svg_for_plants_to_handle()
        elif isinstance(self.system.field_friend.z_axis, Sprayer):
            svg += self.build_svg_for_sprayer_position()
        self.image_view.set_content(svg)

    def detections_to_svg(self, detections: rosys.vision.Detections, *, radius: float = 25, stroke_width: int = 4) -> str:
        svg = ''
        for point in detections.points:
            is_crop = point.category_name in self.plant_locator.crop_category_names
            color = 'green' if is_crop else 'red'
            _radius = (radius - 3) if is_crop else radius
            svg += f'''<circle cx="{int(point.x / self.shrink_factor)}" cy="{int(point.y / self.shrink_factor)}"
                        r="{int(_radius / self.shrink_factor)}" stroke="{color}" fill="none"
                        stroke-dasharray="{int(stroke_width / self.shrink_factor)}"
                        stroke-linejoin="round" stroke-width="{int(stroke_width / self.shrink_factor)}"/>'''
        return svg

    def build_svg_for_plant_provider(self, *, radius: float = 15, stroke_width: int = 3) -> str:
        def draw_plant(plant: Plant, color: str, radius: float) -> str:
            assert self.camera is not None
            assert self.camera.calibration is not None
            plant_2d = self.camera.calibration.project_to_image(plant.position)
            if plant_2d is None:
                return ''
            svg = f'''<circle cx="{int(plant_2d.x/self.shrink_factor)}" cy="{int(plant_2d.y/self.shrink_factor)}"
                        r="{radius/self.shrink_factor}" fill="none"
                        stroke="{color}" stroke-width="{np.ceil(stroke_width/self.shrink_factor)}" />'''
            return svg

        if self.camera is None or self.camera.calibration is None:
            return ''
        position = Point3d(x=self.camera.calibration.extrinsics.translation[0],
                           y=self.camera.calibration.extrinsics.translation[1],
                           z=0).in_frame(self.robot_locator.pose_frame).resolve()
        svg = ''
        for weed in self.plant_provider.get_relevant_weeds(position):
            svg += draw_plant(weed, 'red', radius)
        for crop in self.plant_provider.get_relevant_crops(position):
            svg += draw_plant(crop, 'green', radius-3)
        return svg

    def build_svg_for_tool_position(self, *, radius: float = 10, stroke_width: int = 3) -> str:
        assert self.camera is not None
        assert self.camera.calibration is not None
        assert isinstance(self.field_friend.y_axis, Axis)
        tool_3d = Pose3d(x=self.field_friend.WORK_X, y=self.field_friend.y_axis.position + self.field_friend.WORK_Y, z=0) \
            .in_frame(self.robot_locator.pose_frame).resolve().point_3d
        tool_2d = self.camera.calibration.project_to_image(tool_3d)
        if tool_2d is None:
            return ''
        tool_2d = tool_2d / self.shrink_factor
        return f'''<circle cx="{int(tool_2d.x)}" cy="{int(tool_2d.y)}" r="{int(radius/self.shrink_factor)}"
                    stroke="black" stroke-width="{np.ceil(stroke_width/self.shrink_factor)}" fill="transparent"/>'''

    def build_svg_for_sprayer_position(self, *, radius: float = 10, stroke_width: int = 3) -> str:
        assert self.camera is not None
        assert self.camera.calibration is not None
        assert isinstance(self.field_friend.z_axis, Sprayer)
        tool_3d = Pose3d(x=self.field_friend.WORK_X, y=self.field_friend.WORK_Y, z=0) \
            .in_frame(self.robot_locator.pose_frame).resolve().point_3d
        tool_2d = self.camera.calibration.project_to_image(tool_3d)
        if tool_2d is None:
            return ''
        tool_2d = tool_2d / self.shrink_factor
        return f'''<circle cx="{int(tool_2d.x)}" cy="{int(tool_2d.y)}" r="{int(radius/self.shrink_factor)}"
                    stroke="black" stroke-width="{np.ceil(stroke_width/self.shrink_factor)}" fill="transparent"/>'''

    def build_svg_for_tool_axis(self, *, stroke_width: int = 3) -> str:
        assert self.camera is not None
        assert self.camera.calibration is not None
        assert isinstance(self.field_friend.y_axis, Axis)
        min_tool_3d = Pose3d(x=self.field_friend.WORK_X,
                             y=self.field_friend.y_axis.min_position + self.field_friend.WORK_Y,
                             z=0).in_frame(self.robot_locator.pose_frame).resolve().point_3d
        min_tool_2d = self.camera.calibration.project_to_image(min_tool_3d)
        max_tool_3d = Pose3d(x=self.field_friend.WORK_X,
                             y=self.field_friend.y_axis.max_position + self.field_friend.WORK_Y,
                             z=0).in_frame(self.robot_locator.pose_frame).resolve().point_3d
        max_tool_2d = self.camera.calibration.project_to_image(max_tool_3d)
        if min_tool_2d is None or max_tool_2d is None:
            return ''
        min_tool_2d = min_tool_2d / self.shrink_factor
        max_tool_2d = max_tool_2d / self.shrink_factor
        return f'''<line x1="{int(min_tool_2d.x)}" y1="{int(min_tool_2d.y)}"
                    x2="{int(max_tool_2d.x)}" y2="{int(max_tool_2d.y)}"
                    stroke="black" stroke-width="{np.ceil(stroke_width/self.shrink_factor)}" />'''

    def build_svg_for_plants_to_handle(self) -> str:
        assert self.camera is not None
        assert self.camera.calibration is not None
        assert isinstance(self.system.current_implement, WeedingImplement)
        plants_to_handle = self.system.current_implement.crops_to_handle \
            if isinstance(self.system.current_implement, TornadoImplement) else self.system.current_implement.weeds_to_handle
        svg = ''
        for i, plant_id in enumerate(plants_to_handle.keys()):
            try:
                # TODO: inefficient
                plant = self.plant_provider.get_plant_by_id(plant_id)
            except ValueError:
                continue
            plant_2d = self.camera.calibration.project_to_image(plant.position)
            if plant_2d is not None:
                svg += f'''<circle cx="{int(plant_2d.x/self.shrink_factor)}" cy="{int(plant_2d.y/self.shrink_factor)}"
                            r="{int(8/self.shrink_factor)}" stroke="blue" fill="none" stroke-width="{int(1/self.shrink_factor)}" />'''
                svg += f'''<text x="{int(plant_2d.x/self.shrink_factor)}" y="{int(plant_2d.y/self.shrink_factor) + 4}"
                            fill="blue" font-size="9" text-anchor="middle">{i}</text>'''
        return svg

    def build_svg_for_work_area(self, *, stroke_width: int = 3) -> str:
        # TODO: use nicegui image_layers when they are available
        assert self.camera is not None
        assert self.camera.calibration is not None
        assert self.field_friend.y_axis is not None
        top_point = Point(x=self.camera.calibration.intrinsics.size.width / 2, y=0)
        bottom_point = Point(x=self.camera.calibration.intrinsics.size.width / 2,
                             y=self.camera.calibration.intrinsics.size.height)
        back_point = self.camera.calibration.project_from_image(top_point)
        assert back_point is not None
        back_point = back_point.relative_to(self.robot_locator.pose_frame)
        front_point = self.camera.calibration.project_from_image(bottom_point)
        assert front_point is not None
        front_point = front_point.relative_to(self.robot_locator.pose_frame)
        svg = ''
        starts: tuple[Point, Point] | None = None
        ends: tuple[Point, Point] | None = None
        for x in np.linspace(back_point.x, front_point.x, 5):
            min_3d = Point3d(x=x, y=self.field_friend.y_axis.min_position, z=0.0) \
                .in_frame(self.robot_locator.pose_frame).resolve()
            min_2d = self.camera.calibration.project_to_image(min_3d)
            max_3d = Point3d(x=x, y=self.field_friend.y_axis.max_position, z=0.0) \
                .in_frame(self.robot_locator.pose_frame).resolve()
            max_2d = self.camera.calibration.project_to_image(max_3d)
            if not min_2d or not max_2d:
                continue
            min_2d = min_2d / self.shrink_factor
            max_2d = max_2d / self.shrink_factor
            if starts is None:
                starts = (min_2d, max_2d)
                continue
            ends = (min_2d, max_2d)
            svg += f'''<line x1="{int(starts[0].x)}" y1="{int(starts[0].y)}"
                        x2="{int(ends[0].x)}" y2="{int(ends[0].y)}"
                        stroke="black" stroke-width="{np.ceil(stroke_width/self.shrink_factor)}" />'''
            svg += f'''<line x1="{int(starts[1].x)}" y1="{int(starts[1].y)}"
                        x2="{int(ends[1].x)}" y2="{int(ends[1].y)}"
                        stroke="black" stroke-width="{np.ceil(stroke_width/self.shrink_factor)}" />'''
            starts = (min_2d, max_2d)
        return svg

    def build_svg_for_mapping(self, *, track_width: float = 0.10, radius: float = 3, stroke_width: int = 2) -> str:
        def draw_point(point: Point, color: str) -> str:
            return f'''<circle cx="{int(point.x / self.shrink_factor)}" cy="{int(point.y / self.shrink_factor)}"
                        r="{int(radius / self.shrink_factor)}" fill="none"
                        stroke="{color}" stroke-width="{np.ceil(stroke_width / self.shrink_factor)}" />'''

        assert self.camera is not None
        assert self.camera.calibration is not None
        top_point = Point(x=self.camera.calibration.intrinsics.size.width / 2, y=0)
        bottom_point = Point(x=self.camera.calibration.intrinsics.size.width / 2,
                             y=self.camera.calibration.intrinsics.size.height)
        back_point = self.camera.calibration.project_from_image(top_point)
        assert back_point is not None
        back_point = back_point.relative_to(self.robot_locator.pose_frame)
        front_point = self.camera.calibration.project_from_image(bottom_point)
        assert front_point is not None
        front_point = front_point.relative_to(self.robot_locator.pose_frame)
        y_range = self.system.config.measurements.wheel_distance / 2 - track_width / 2
        world_points = [Point3d(x=x, y=y, z=0).in_frame(self.robot_locator.pose_frame).resolve()
                        for x in np.linspace(back_point.x, front_point.x, 20)
                        for y in np.linspace(-y_range, y_range, 20)]
        image_points = self.camera.calibration.project_to_image(world_points)
        colors_rgb = [colorsys.hsv_to_rgb(f, 1, 1) for f in np.linspace(0, 1, len(world_points))]
        colors_hex = [f'#{int(rgb[0] * 255):02x}{int(rgb[1] * 255):02x}{int(rgb[2] * 255):02x}' for rgb in colors_rgb]
        return ''.join(draw_point(p, color) for p, color in zip(image_points, colors_hex, strict=False) if p is not None)
