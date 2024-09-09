import rosys
from rosys.testing import assert_point, forward

from field_friend import System


async def test_locating_of_plants(system: System, detector: rosys.vision.DetectorSimulation):
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='sugar_beet',
                                                                   position=rosys.geometry.Point3d(x=0.212, y=0.03, z=0)))
    system.automator.start(system.straight_line_navigation.start())
    await forward(20)
    assert len(system.plant_provider.crops) == 1
    assert system.plant_provider.crops[0].type == 'sugar_beet'
    assert_point(system.plant_provider.crops[0].position, rosys.geometry.Point3d(x=0.212, y=0.03, z=0))
