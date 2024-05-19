from typing import TYPE_CHECKING

import rosys
from nicegui.elements.scene_object3d import Object3D
from nicegui.elements.scene_objects import Curve

if TYPE_CHECKING:
    from ... import System


class visualizer_object(Object3D):
    def __init__(self, system: 'System') -> None:
        super().__init__('group')
        self.system = system
        self.system.path_provider.SHOW_PATH.register_ui(self.update_path)
        self.system.mowing.MOWING_STARTED.register_ui(self.refresh)
        self.system.weeding.PATH_PLANNED.register_ui(self.refresh)
        self.system.AUTOMATION_CHANGED.register_ui(lambda _: self.refresh())
        self.refresh()

    def refresh(self) -> None:
        automation = self.system.get_current_automation_id()
        if automation == 'weeding':
            segments = self.system.weeding.weeding_plan + self.system.weeding.turn_paths
        elif automation == 'mowing':
            segments = self.system.mowing.paths
        else:
            segments = []
        self.update_path([path_segment for path in segments for path_segment in path])

    def update_path(self, path: list[rosys.driving.PathSegment], height: float = 0.2) -> None:
        for obj in list(self.scene.objects.values()):
            if obj.name == 'path':
                obj.delete()
        for segment in path:
            Curve(
                [segment.spline.start.x, segment.spline.start.y, height],
                [segment.spline.control1.x, segment.spline.control1.y, height],
                [segment.spline.control2.x, segment.spline.control2.y, height],
                [segment.spline.end.x, segment.spline.end.y, height],
            ).material('#ff8800').with_name('path')

    def clear(self) -> None:
        self.update_path([])
