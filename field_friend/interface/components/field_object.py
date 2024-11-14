from typing import TYPE_CHECKING

import numpy as np
from nicegui.elements.scene_objects import Box, Curve, Cylinder, Group
from rosys.geometry import Spline

from ...automations import Field, FieldProvider
from ...automations.navigation import FieldNavigation

if TYPE_CHECKING:
    from field_friend.system import System


class field_object(Group):

    def __init__(self, system: 'System') -> None:
        super().__init__()
        self.system = system
        self.field_provider: FieldProvider = system.field_provider
        self._update()
        self.field_provider.FIELDS_CHANGED.register_ui(self._update)
        self.field_provider.FIELD_SELECTED.register_ui(self._update)

    def create_fence(self, start, end):
        height = 0.12
        depth = 0.05
        # Calculate the center point of the plank
        center_x = (start[0] + end[0]) / 2
        center_y = (start[1] + end[1]) / 2

        # Calculate the length of the plank (distance between start and end)
        length = np.sqrt((end[0] - start[0]) ** 2 + (end[1] - start[1]) ** 2)

        # Calculate the angle of the plank
        angle = np.arctan2(end[1] - start[1], end[0] - start[0])

        # Create and return the plank object
        Box(length, height, depth).move(x=center_x, y=center_y,
                                        z=height / 2 + 0.2).with_name('field_').material('#8b4513').rotate(np.pi/2, 0, angle)
        Box(length, height, depth).move(x=center_x, y=center_y,
                                        z=height / 2 + 0.5).with_name('field_').material('#8b4513').rotate(np.pi/2, 0, angle)
        Box(length, height, depth).move(x=center_x, y=center_y,
                                        z=height / 2 + 0.8).with_name('field_').material('#8b4513').rotate(np.pi/2, 0, angle)
        Cylinder(0.1, 0.1, 1.0).move(x=start[0], y=start[1], z=0.5).with_name(
            'field_').material('#8b4513').rotate(np.pi/2, 0, 0)
        Cylinder(0.1, 0.1, 1.0).move(x=end[0], y=end[1], z=0.5).with_name(
            'field_').material('#8b4513').rotate(np.pi/2, 0, 0)

    def _update(self) -> None:
        self.update(self.system.field_provider.selected_field)

    def update(self, active_field: Field | None) -> None:
        [obj.delete() for obj in list(self.scene.objects.values()) if obj.name and obj.name.startswith('field_')]
        [obj.delete() for obj in list(self.scene.objects.values()) if obj.name and obj.name.startswith('row_')]
        if active_field is None:
            return
        outline = [point.cartesian().tuple for point in active_field.outline]
        if len(outline) > 1:  # Make sure there are at least two points to form a segment
            for i in range(len(outline)):
                start = outline[i]
                end = outline[(i + 1) % len(outline)]  # Loop back to the first point
                self.create_fence(start, end)

            for row in active_field.rows:
                if len(row.points) == 1:
                    continue
                else:
                    row_points = row.cartesian()
                    for i in range(len(row_points) - 1):
                        spline = Spline.from_points(row_points[i], row_points[i + 1])
                        Curve(
                            [spline.start.x, spline.start.y, 0],
                            [spline.control1.x, spline.control1.y, 0],
                            [spline.control2.x, spline.control2.y, 0],
                            [spline.end.x, spline.end.y, 0],
                        ).material('#6c541e').with_name(f'row_{row.id}_{i}')

            for point in row_points:
                Cylinder(0.04, 0.04, 0.7).move(x=point.x, y=point.y, z=0.35).material(
                    'black').with_name(f'row_{row.id}_point').rotate(np.pi / 2, 0, 0)
