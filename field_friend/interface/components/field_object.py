from __future__ import annotations

from itertools import pairwise
from typing import TYPE_CHECKING

import numpy as np
from nicegui.elements.scene_objects import Box, Curve, Cylinder, Group, Sphere
from rosys.geometry import Point, Spline

from ...automations import Field, FieldProvider

if TYPE_CHECKING:
    from ...system import System


class FieldObject(Group):

    def __init__(self, system: System) -> None:
        super().__init__()
        self.system = system
        self.field_provider: FieldProvider = system.field_provider
        self._update()
        self.field_provider.FIELDS_CHANGED.subscribe(self._update)
        self.field_provider.FIELD_SELECTED.subscribe(self._update)
        self.system.GNSS_REFERENCE_CHANGED.subscribe(self._update)

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
        Box(length, height, depth).move(x=center_x, y=center_y, z=height / 2 + 0.2) \
            .with_name('field_').material('#8b4513').rotate(np.pi/2, 0, angle)
        Box(length, height, depth).move(x=center_x, y=center_y, z=height / 2 + 0.5) \
            .with_name('field_').material('#8b4513').rotate(np.pi/2, 0, angle)
        Box(length, height, depth).move(x=center_x, y=center_y, z=height / 2 + 0.8) \
            .with_name('field_').material('#8b4513').rotate(np.pi/2, 0, angle)
        Cylinder(0.1, 0.1, 1.0).move(x=start[0], y=start[1], z=0.5) \
            .with_name('field_').material('#8b4513').rotate(np.pi/2, 0, 0)
        Cylinder(0.1, 0.1, 1.0).move(x=end[0], y=end[1], z=0.5) \
            .with_name('field_').material('#8b4513').rotate(np.pi/2, 0, 0)

    def _update(self) -> None:
        self.update(self.system.field_provider.selected_field)

    def update(self, active_field: Field | None) -> None:
        # TODO: now we have empty keys in our objects dict. Is this intended?
        for obj in list(self.scene.objects.values()):
            if obj.name and obj.name.startswith(('field_', 'row_', 'bed_', 'docking_')):
                obj.delete()

        if active_field is None:
            return
        outline = [point.to_local().tuple for point in active_field.outline]
        if len(outline) <= 1:  # Make sure there are at least two points to form a segment
            return
        for i, start in enumerate(outline):
            end = outline[(i + 1) % len(outline)]  # Loop back to the first point
            self.create_fence(start, end)

        for row_index, row in enumerate(active_field.rows):
            if len(row.points) == 1:
                continue
            row_points: list[Point] = [point.to_local() for point in row.points]
            for i, (point1, point2) in enumerate(pairwise(row_points)):
                spline = Spline.from_points(point1, point2)
                Curve(
                    [spline.start.x, spline.start.y, 0],
                    [spline.control1.x, spline.control1.y, 0],
                    [spline.control2.x, spline.control2.y, 0],
                    [spline.end.x, spline.end.y, 0],
                ).material('#6c541e').with_name(f'row_{row.id}_{i}')
            bed_row_name = str(int(row.name.replace('row_', '')) % active_field.row_count)
            self.scene.text(bed_row_name, style='font-size: 0.6em;') \
                .move(x=row_points[0].x, y=row_points[0].y, z=0.01).with_name(f'{row.name}_label_start')
            self.scene.text(bed_row_name, style='font-size: 0.6em;') \
                .move(x=row_points[-1].x, y=row_points[-1].y, z=0.01).with_name(f'{row.name}_label_end')

            if row_index % active_field.row_count == 0:
                row_direction = row_points[0].direction(row_points[-1])
                bed_point = row_points[0].polar(-0.5, row_direction)
                bed_index = f'{int(row_index / active_field.row_count)}'
                self.scene.text('Bed ' + bed_index, style='font-size: 0.6em;') \
                    .move(x=bed_point.x, y=bed_point.y, z=0.01).with_name(f'bed_{bed_index}_label')

        if active_field.charge_dock_pose is not None:
            assert active_field.charge_approach_pose is not None
            local_docked_pose = active_field.charge_dock_pose.to_local()
            Sphere(radius=0.05).move(x=local_docked_pose.x, y=local_docked_pose.y, z=0.1) \
                .material('#ff0000').with_name('docking_station')
            local_approach_pose = active_field.charge_approach_pose.to_local()
            Sphere(radius=0.05).move(x=local_approach_pose.x, y=local_approach_pose.y, z=0.1) \
                .material('#008000').with_name('docking_approach')
