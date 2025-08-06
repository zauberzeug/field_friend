from __future__ import annotations

from contextlib import nullcontext
from typing import TYPE_CHECKING

from nicegui import ui
from nicegui.elements.scene_objects import Curve

from ...automations.navigation import DriveSegment

if TYPE_CHECKING:
    from ... import System


class PathObject(ui.scene.group):
    """
    A path object that displays the upcoming path of the robot.

    Based on https://github.com/zauberzeug/rosys/blob/main/rosys/pathplanning/path_object_.py
    """

    def __init__(self, system: System, *, height: float = 0.05) -> None:
        super().__init__()
        self.system = system
        self.height = height
        self.system.automator.AUTOMATION_STARTED.register_ui(self.register)
        self.system.automator.AUTOMATION_STOPPED.register_ui(lambda _: self.clear_path())

    def register(self) -> None:
        if self.system.current_navigation is None:
            return

        def update_upcoming_path() -> None:
            assert self.system.current_navigation is not None
            self.update(self.system.current_navigation.path)
        self.system.current_navigation.WAYPOINT_REACHED.register_ui(update_upcoming_path)
        self.system.current_navigation.PATH_GENERATED.register_ui(self.update)

    def update(self, path: list[DriveSegment]) -> None:
        self.clear_path()
        with self.scene or nullcontext():
            for segment in path:
                color: str
                if segment.use_implement:
                    color = '#ff0000'  # red
                else:
                    color = '#87ceeb'  # light blue
                Curve(
                    [segment.spline.start.x, segment.spline.start.y, self.height],
                    [segment.spline.control1.x, segment.spline.control1.y, self.height],
                    [segment.spline.control2.x, segment.spline.control2.y, self.height],
                    [segment.spline.end.x, segment.spline.end.y, self.height],
                ).material(color).with_name('path')

    def clear_path(self) -> None:
        for obj in list(self.scene.objects.values()):
            if obj.name == 'path':
                obj.delete()
