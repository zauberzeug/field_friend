from copy import deepcopy

import rosys
from rosys.testing import forward

from field_friend import System


async def test_working_with_weed_screw(system: System, detector: rosys.vision.DetectorSimulation):
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='maize',
                                                                   position=rosys.geometry.Point3d(x=0.2, y=0.0, z=0)))
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='thistle',
                                                                   position=rosys.geometry.Point3d(x=0.2, y=0.05, z=0)))
    system.straight_line_navigation.implement = system.tools['Weed Screw']
    system.automator.start(system.straight_line_navigation.start())
    await forward(20)
    assert len(detector.simulated_objects) == 1
    assert detector.simulated_objects[0].category_name == 'maize'


async def test_keep_crops_safe(system: System, detector: rosys.vision.DetectorSimulation):
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='maize',
                                                                   position=rosys.geometry.Point3d(x=0.2, y=0.0, z=0)))
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='thistle',
                                                                   position=rosys.geometry.Point3d(x=0.2, y=system.field_friend.DRILL_RADIUS-0.01, z=0)))
    system.straight_line_navigation.implement = system.tools['Weed Screw']
    system.automator.start(system.straight_line_navigation.start())
    await forward(20)
    assert len(detector.simulated_objects) == 1
    assert detector.simulated_objects[0].category_name == 'maize'
