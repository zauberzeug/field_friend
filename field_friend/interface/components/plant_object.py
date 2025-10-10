import logging
from typing import TYPE_CHECKING

import rosys
from nicegui.elements.scene_objects import Group, Sphere

if TYPE_CHECKING:
    from ...system import System


class PlantObjects(Group):

    def __init__(self, system: 'System') -> None:
        super().__init__()

        self.plant_provider = system.plant_provider
        self.plant_locator = system.plant_locator
        self.log = logging.getLogger('field_friend.plant_objects')
        self.update()
        self.plant_provider.PLANTS_CHANGED.subscribe(self.update)

    def update(self) -> None:
        origin = rosys.geometry.Point3d(x=0, y=0, z=0)
        in_world = {p.id: p for p in
                    self.plant_provider.get_relevant_weeds(origin, max_distance=1000) +
                    self.plant_provider.get_relevant_crops(origin, max_distance=1000)}
        rendered = {o.name.split(':')[1]: o for o in self.scene.objects.values()
                    if o.name and o.name.startswith('plant_')}
        for id_, obj in rendered.items():
            if id_ not in in_world:
                obj.delete()
        for id_, plant in in_world.items():
            if id_ not in rendered:
                if plant.type in self.plant_locator.weed_category_names:
                    Sphere(0.02).with_name(f'plant_{plant.type}:{id_}') \
                        .material('#ef1208') \
                        .move(plant.position.x, plant.position.y, 0.02)
                else:
                    Sphere(0.035).with_name(f'plant_{plant.type}:{id_}') \
                        .material('#11ede3') \
                        .move(plant.position.x, plant.position.y, 0.035)
