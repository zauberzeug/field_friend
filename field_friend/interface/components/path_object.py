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
        with self.scene or nullcontext():
            self.system.automator.AUTOMATION_STARTED.subscribe(self._subscribe)
            self.system.automator.AUTOMATION_STOPPED.subscribe(lambda _: self._clear_path())

    def _subscribe(self) -> None:
        def update_upcoming_path(_: DriveSegment) -> None:
            assert self.system.current_navigation is not None
            self._update(self.system.current_navigation.path)
        if self.system.current_navigation is None:
            return
        with self.scene or nullcontext():
            self.system.current_navigation.SEGMENT_COMPLETED.subscribe(update_upcoming_path)
            self.system.current_navigation.PATH_GENERATED.subscribe(self._update)

    def _update(self, path: list[DriveSegment]) -> None:
        self._clear_path()
        with self.scene or nullcontext():
            for segment in reversed(path):
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

    def _clear_path(self) -> None:
        for obj in list(self.scene.objects.values()):
            if obj.name == 'path':
                obj.delete()
