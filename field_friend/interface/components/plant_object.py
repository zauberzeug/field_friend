import logging
from nicegui.elements.scene_objects import Group, Sphere

from ...automations import PlantProvider

class plant_objects(Group):

    def __init__(self, plant_provider: PlantProvider, weed_category_names: list[str]) -> None:
        super().__init__()

        self.plant_provider = plant_provider
        self.weed_category_names = weed_category_names
        self.log = logging.getLogger('field_friend.plant_objects')
        self.update()
        self.plant_provider.PLANTS_CHANGED.register_ui(self.update)

    def update(self) -> None:
        in_world = {p.id: p for p in self.plant_provider.weeds+self.plant_provider.crops}
        rendered = {o.name.split(':')[1]: o for o in self.scene.objects.values()
                    if o.name and o.name.startswith('plant_')}
        for id, obj in rendered.items():
            if id not in in_world:
                obj.delete()
        for id, plant in in_world.items():
            if id not in rendered:
                Sphere(0.02).with_name(f'plant_{plant.type}:{id}') \
                    .material('#ef1208' if plant.type in self.weed_category_names else '#11ede3') \
                    .move(plant.position.x, plant.position.y, 0.02)
