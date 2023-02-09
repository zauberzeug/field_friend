import logging

from nicegui import ui
from nicegui.elements.scene_object3d import Object3D
from nicegui.elements.scene_objects import Sphere

from ..automations import PlantProvider


class plant_objects(Object3D):

    def __init__(self, plant_provider: PlantProvider) -> None:
        super().__init__('group')

        self.plant_provider = plant_provider
        self.log = logging.getLogger('field_friend.plant_objects')
        self.update()
        ui.timer(0.5, self.update)

    def update(self) -> None:
        in_world = {p.id: p for p in self.plant_provider.weeds+self.plant_provider.crops}
        rendered = {o.name.split('_')[1]: o for o in self.scene.objects.values()
                    if o.name and o.name.startswith('plant_')}
        for id, obj in rendered.items():
            if id not in in_world:
                obj.delete()
        for id, plant in in_world.items():
            if id not in rendered:
                if plant.type == 'weed':
                    Sphere(0.02).material('#ef1208').move(plant.position.x, plant.position.y, 0.02) \
                        .with_name(f'plant_weed_{id}')
                else:
                    Sphere(0.02).material('#11ede3').move(plant.position.x, plant.position.y, 0.02) \
                        .with_name(f'plant_crop_{id}')
