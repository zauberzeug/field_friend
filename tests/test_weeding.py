import pytest
import rosys
from rosys.testing import forward

from field_friend import System
from field_friend.automations.implements import Tornado


async def test_working_with_weeding_screw(system: System, detector: rosys.vision.DetectorSimulation):
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='maize',
                                                                   position=rosys.geometry.Point3d(x=0.2, y=0.0, z=0)))
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='thistle',
                                                                   position=rosys.geometry.Point3d(x=0.2, y=0.05, z=0)))
    system.current_implement = system.implements['Weed Screw']
    system.current_navigation = system.straight_line_navigation
    system.automator.start()
    await forward(20)
    assert len(detector.simulated_objects) == 1
    assert detector.simulated_objects[0].category_name == 'maize'


async def test_keep_crops_safe(system: System, detector: rosys.vision.DetectorSimulation):
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='maize',
                                                                   position=rosys.geometry.Point3d(x=0.2, y=0.0, z=0)))
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='thistle',
                                                                   position=rosys.geometry.Point3d(x=0.2, y=system.field_friend.DRILL_RADIUS-0.01, z=0)))
    system.current_implement = system.implements['Weed Screw']
    system.current_navigation = system.straight_line_navigation
    system.automator.start()
    await forward(20)
    assert len(detector.simulated_objects) == 1
    assert detector.simulated_objects[0].category_name == 'maize'


async def test_weeding_screw_only_targets_big_weed(system: System, detector: rosys.vision.DetectorSimulation):
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='weed',
                                                                   position=rosys.geometry.Point3d(x=0.2, y=0.0, z=0)))
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='thistle',
                                                                   position=rosys.geometry.Point3d(x=0.15, y=0, z=0)))
    system.current_implement = system.implements['Weed Screw']
    system.current_navigation = system.straight_line_navigation
    system.automator.start()
    await forward(20)
    assert len(detector.simulated_objects) == 1
    assert detector.simulated_objects[0].category_name == 'weed'


@pytest.mark.parametrize('system', ['rb28'], indirect=True)
async def test_tornado_removes_weeds_around_crop(system: System, detector: rosys.vision.DetectorSimulation):
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='sugar_beet',
                                                                   position=rosys.geometry.Point3d(x=0.2, y=0.0, z=0)))
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='weed',
                                                                   position=rosys.geometry.Point3d(x=0.23, y=0.0, z=0)))
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='weed',
                                                                   position=rosys.geometry.Point3d(x=0.2, y=0.05, z=0)))
    system.current_implement = system.implements['Tornado']
    system.current_navigation = system.straight_line_navigation
    system.automator.start()
    await forward(30)
    assert len(detector.simulated_objects) == 1
    assert detector.simulated_objects[0].category_name == 'sugar_beet'


@pytest.mark.skip(reason='Tornado does not yet remove weeds between crops')
@pytest.mark.parametrize('system', ['rb28'], indirect=True)
async def test_tornado_removes_weeds_between_crops(system: System, detector: rosys.vision.DetectorSimulation):
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='sugar_beet',
                                                                   position=rosys.geometry.Point3d(x=0.1, y=0.0, z=0)))
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='sugar_beet',
                                                                   position=rosys.geometry.Point3d(x=0.3, y=0.0, z=0)))
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='weed',
                                                                   position=rosys.geometry.Point3d(x=0.2, y=0.0, z=0)))
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='weed',
                                                                   position=rosys.geometry.Point3d(x=0.2, y=0.05, z=0)))
    tornado = system.implements['Tornado']
    assert isinstance(tornado, Tornado)
    tornado.drill_between_crops = True
    system.current_implement = tornado
    system.automator.start(system.straight_line_navigation.start())
    await forward(30)
    assert len(detector.simulated_objects) == 2
    assert detector.simulated_objects[0].category_name == 'sugar_beet'
    assert detector.simulated_objects[1].category_name == 'sugar_beet'
