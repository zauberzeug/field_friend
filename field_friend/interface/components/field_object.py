import numpy as np
from nicegui.elements.scene_objects import Box, Curve, Cylinder, Extrusion, Group
from rosys.geometry import Spline
from ...automations import FieldProvider, Field


class field_object(Group):

    def __init__(self, field_provider: FieldProvider, active_field: Field | None) -> None:
        super().__init__()

        self.field_provider = field_provider
        self.update(active_field)
        self.field_provider.FIELDS_CHANGED.register_ui(lambda: self.update(active_field))

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
                                        z=height / 2 + 0.5).with_name('field_').material('#8b4513').rotate(np.pi/2, 0, angle)  # Convert angle from radians to degrees
        Box(length, height, depth).move(x=center_x, y=center_y,
                                        z=height / 2 + 0.8).with_name('field_').material('#8b4513').rotate(np.pi/2, 0, angle)
        Cylinder(0.1, 0.1, 1.0).move(x=start[0], y=start[1], z=0.5).with_name(
            'field_').material('#8b4513').rotate(np.pi/2, 0, 0)
        Cylinder(0.1, 0.1, 1.0).move(x=end[0], y=end[1], z=0.5).with_name(
            'field_').material('#8b4513').rotate(np.pi/2, 0, 0)

    def update(self, active_field: Field | None) -> None:
        [obj.delete() for obj in list(self.scene.objects.values()) if obj.name and obj.name.startswith('field_')]
        [obj.delete() for obj in list(self.scene.objects.values()) if obj.name and obj.name.startswith('obstacle_')]
        [obj.delete() for obj in list(self.scene.objects.values()) if obj.name and obj.name.startswith('row_')]
        if active_field and active_field.reference is not None:
            field = active_field
            outline = [[point.x, point.y] for point in field.outline]
            if len(outline) > 1:  # Make sure there are at least two points to form a segment
                for i in range(len(outline)):
                    start = outline[i]
                    end = outline[(i + 1) % len(outline)]  # Loop back to the first point
                    self.create_fence(start, end)

            if not field.reference:
                return
            for obstacle in field.obstacles:
                outline = [[point.x, point.y] for point in obstacle.cartesian(field.reference)]
                Extrusion(outline, 0.1).with_name(f'obstacle_{obstacle.id}').material('#B80F0A')

            for row in field.rows:
                if len(row.points) == 1:
                    continue
                else:
                    row_points = row.cartesian(field.reference)
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
