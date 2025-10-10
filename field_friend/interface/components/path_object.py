from __future__ import annotations

from typing import TYPE_CHECKING, Self

from nicegui.elements.scene_objects import Curve, Group

from ...automations.navigation import DriveSegment

if TYPE_CHECKING:
    from ... import System


class PathObject(Group):
    """
    A path object that displays the upcoming path of the robot.

    Based on https://github.com/zauberzeug/rosys/blob/main/rosys/pathplanning/path_object_.py
    """

    def __init__(self, system: System, *, height: float = 0.05, visible: bool = True) -> None:
        super().__init__()
        self.system = system
        self.height = height
        assert self.system.current_navigation is not None
        self.clear_path()
        self.visible(visible)

    def visible(self, value: bool = True) -> Self:
        assert self.system.current_navigation is not None
        if value:
            self.update(self.system.current_navigation.path)
            with self:
                self.system.current_navigation.SEGMENT_COMPLETED.register_ui(self.update_upcoming_segment)
                self.system.current_navigation.PATH_GENERATED.register_ui(self.update)
        else:
            self.system.current_navigation.SEGMENT_COMPLETED.unregister(self.update_upcoming_segment)
            self.system.current_navigation.PATH_GENERATED.unregister(self.update)
        super().visible(value)
        return self

    def update(self, path: list[DriveSegment]) -> None:
        self.clear_path()
        with self:
            for segment in reversed(path):
                # NOTE: red for implement, light blue for no implement
                color = '#ff0000' if segment.use_implement else '#87ceeb'
                Curve(
                    [segment.spline.start.x, segment.spline.start.y, self.height],
                    [segment.spline.control1.x, segment.spline.control1.y, self.height],
                    [segment.spline.control2.x, segment.spline.control2.y, self.height],
                    [segment.spline.end.x, segment.spline.end.y, self.height],
                ).material(color).with_name('path')

    def update_upcoming_segment(self, _: DriveSegment | None) -> None:
        # NOTE: just a wrapper for update, because SEGMENT_COMPLETED has a payload
        with self:
            assert self.system.current_navigation is not None
            self.update(self.system.current_navigation.path)

    def clear_path(self) -> None:
        with self:
            for obj in list(self.scene.objects.values()):
                if obj.name == 'path':
                    obj.delete()
