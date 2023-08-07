import numpy as np
from nicegui.elements.scene_objects import Extrusion, Group, Line, Sphere

from ..navigation import FieldProvider


class field_object(Group):

    def __init__(self, field_provider: FieldProvider) -> None:
        super().__init__()

        self.field_provider = field_provider
        self.update()
        self.field_provider.FIELDS_CHANGED.register_ui(self.update)

    def update(self) -> None:
        [obj.delete() for obj in list(self.scene.objects.values()) if obj.name and obj.name.startswith('field_')]
        [obj.delete() for obj in list(self.scene.objects.values()) if obj.name and obj.name.startswith('obstacle_')]
        with self.scene:
            for field in self.field_provider.fields:
                if len(field.outline) == 1:
                    outline = [[field.outline[0].x + 0.05 * np.cos(phi), field.outline[0].y + 0.05 * np.sin(phi)]
                               for phi in np.linspace(0, 2 * np.pi, 16, endpoint=False)]
                else:
                    outline = [[point.x, point.y] for point in field.outline]
                with self.scene:
                    Extrusion(outline, 0.5, wireframe=True).with_name(f'field_{field.name}').material('#63462D')
                # for i, point in enumerate(field.outline):
                #     if i == 0:
                #         Sphere(0.05).material('#53B689').with_name(f'field').move(x=point.x, y=point.y, z=0.02)
                #     else:
                #         Sphere(0.05).material('#63462D').with_name(f'field').move(x=point.x, y=point.y, z=0.02)
                # for i in range(len(field.outline)-1):
                #     if len(field.outline) < 3:
                #         break
                #     if i == 0:
                #         Line(
                #             [field.outline[i].x, field.outline[i].y, 0.0],
                #             [field.outline[i + 1].x, field.outline[i + 1].y, 0.0],
                #         ).material('#53B689').with_name(f'field')
                #     else:
                #         Line(
                #             [field.outline[i].x, field.outline[i].y, 0.0],
                #             [field.outline[i + 1].x, field.outline[i + 1].y, 0.0],
                #         ).material('#63462D').with_name(f'field')
                #     Line(
                #         [field.outline[-1].x, field.outline[-1].y, 0.0],
                #         [field.outline[0].x, field.outline[0].y, 0.0],
                #     ).material('#63462D').with_name(f'field')
                for obstacle in field.obstacles:
                    outline = [[point.x, point.y] for point in obstacle.points]
                    Extrusion(outline, 0.1).with_name(f'obstacle_{obstacle.name}').material('#B80F0A')
                    # for point in obstacle.points:
                    #     Sphere(0.05).material('#63462D').with_name(f'field').move(x=point.x, y=point.y, z=0.02)
                    # for i in range(len(obstacle.points) - 1):
                    #     if len(obstacle.points) < 3:
                    #         break
                    #     Line(
                    #         [obstacle.points[i].x, obstacle.points[i].y, 0.0],
                    #         [obstacle.points[i + 1].x, obstacle.points[i + 1].y, 0.0],
                    #     ).material('red').with_name(f'field')
                    #     Line(
                    #         [obstacle.points[-1].x, obstacle.points[-1].y, 0.0],
                    #         [obstacle.points[0].x, obstacle.points[0].y, 0.0],
                    # ).material('red').with_name(f'field')
