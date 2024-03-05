from typing import Optional

from nicegui.elements.scene_object3d import Object3D
from nicegui.elements.scene_objects import Curve
from rosys.automation import Automator
from rosys.driving import PathSegment

from ..automations import Mowing, PathProvider, Weeding


class visualizer_object(Object3D):
    def __init__(
            self, automator: Automator, path_provider: Optional[PathProvider],
            mowing: Optional[Mowing] = None, weeding: Optional[Weeding] = None) -> None:
        super().__init__('group')
        self.mowing = mowing
        self.path_provider = path_provider
        self.automator = automator
        self.weeding = weeding

        if self.path_provider:
            self.path_provider.SHOW_PATH.register_ui(self.update_path)
        if self.mowing:
            self.mowing.MOWING_STARTED.register_ui(self.update_path)
        if self.weeding:
            self.weeding.PATH_PLANNED.register_ui(self.update_path)

    def update_path(self, path: list[PathSegment], height: float = 0.2) -> None:
        [obj.delete() for obj in list(self.scene.objects.values()) if obj.name == 'path']
        for segment in path:
            Curve(
                [segment.spline.start.x, segment.spline.start.y, height],
                [segment.spline.control1.x, segment.spline.control1.y, height],
                [segment.spline.control2.x, segment.spline.control2.y, height],
                [segment.spline.end.x, segment.spline.end.y, height],
            ).material('#ff8800').with_name('path')

    def clear(self) -> None:
        self.update_path([])
