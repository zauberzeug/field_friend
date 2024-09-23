import pytest
import rosys
from rosys.testing import forward

from field_friend import System
from field_friend.automations.implements import Tornado, WeedingScrew


async def test_working_with_weeding_screw(system: System, detector: rosys.vision.DetectorSimulation):
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='maize',
                                                                   position=rosys.geometry.Point3d(x=0.2, y=0.0, z=0)))
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='big_weed',
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
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='big_weed',
                                                                   position=rosys.geometry.Point3d(x=0.2, y=system.field_friend.DRILL_RADIUS-0.01, z=0)))
    system.current_implement = system.implements['Weed Screw']
    system.current_navigation = system.straight_line_navigation
    system.automator.start()
    await forward(20)
    assert len(detector.simulated_objects) == 1
    assert detector.simulated_objects[0].category_name == 'maize'


@pytest.mark.skip(reason='Needs to be rewritten to ignore specific weeds')
async def test_weeding_screw_only_targets_big_weed(system: System, detector: rosys.vision.DetectorSimulation):
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='weed',
                                                                   position=rosys.geometry.Point3d(x=0.2, y=0.0, z=0)))
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='big_weed',
                                                                   position=rosys.geometry.Point3d(x=0.15, y=0, z=0)))
    system.current_implement = system.implements['Weed Screw']
    system.current_navigation = system.straight_line_navigation
    system.automator.start()
    await forward(20)
    assert len(detector.simulated_objects) == 1
    assert detector.simulated_objects[0].category_name == 'weed'


async def test_weeding_screw_does_not_skip_close_weed(system: System, detector: rosys.vision.DetectorSimulation):
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='big_weed',
                                                                   position=rosys.geometry.Point3d(x=0.2, y=0, z=0)))
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='big_weed',
                                                                   position=rosys.geometry.Point3d(x=0.2+system.field_friend.DRILL_RADIUS-0.01, y=0.05, z=0)))
    system.current_implement = system.implements['Weed Screw']
    system.current_navigation = system.straight_line_navigation
    system.automator.start()
    await forward(40)
    assert len(detector.simulated_objects) == 0


async def test_weeding_screw_focus_on_weed_close_to_crop(system: System, detector: rosys.vision.DetectorSimulation):
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='maize',
                                                                   position=rosys.geometry.Point3d(x=0.2, y=0.0, z=0)))
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='big_weed',
                                                                   position=rosys.geometry.Point3d(x=0.1, y=0, z=0)))
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='big_weed',
                                                                   position=rosys.geometry.Point3d(x=0.16, y=0, z=0)))
    system.current_implement = system.implements['Weed Screw']
    system.current_navigation = system.straight_line_navigation
    assert isinstance(system.current_implement, WeedingScrew)
    system.current_implement.cultivated_crop = 'maize'
    system.current_implement.max_crop_distance = 0.05
    system.automator.start()
    await forward(40)
    assert len(detector.simulated_objects) == 2
    assert detector.simulated_objects[1].category_name == 'big_weed'
    assert detector.simulated_objects[1].position.x == 0.1


@pytest.mark.parametrize('cultivated_crop', ['maize', None])
async def test_weeding_screw_advances_when_there_are_no_plants(system: System, detector: rosys.vision.DetectorSimulation, cultivated_crop: str | None):
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='weed',
                                                                   position=rosys.geometry.Point3d(x=0.2, y=-0.05, z=0)))
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='weed',
                                                                   position=rosys.geometry.Point3d(x=1.290, y=-0.04, z=0)))
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='maize',
                                                                   position=rosys.geometry.Point3d(x=1.285, y=0, z=0)))
    assert detector.simulated_objects[1].category_name == 'weed'
    system.current_implement = system.implements['Weed Screw']
    system.current_navigation = system.straight_line_navigation
    system.current_navigation.length = 1.5
    assert isinstance(system.current_implement, WeedingScrew)
    system.current_implement.cultivated_crop = cultivated_crop
    # TODO: test fails if not forwarded by 1 second
    await forward(1)
    system.automator.start()
    await forward(60)
    assert system.odometer.prediction.point.x == pytest.approx(system.straight_line_navigation.length, abs=0.1)
    if cultivated_crop is None:
        assert len(detector.simulated_objects) == 1
    elif cultivated_crop == 'maize':
        assert len(detector.simulated_objects) == 2
    assert detector.simulated_objects[-1].category_name == 'maize'


async def test_weeding_screw_advances_when_there_are_no_weeds_close_enough_to_the_crop(system: System, detector: rosys.vision.DetectorSimulation):
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='weed',
                                                                   position=rosys.geometry.Point3d(x=0.2, y=-0.08, z=0)))
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='maize',
                                                                   position=rosys.geometry.Point3d(x=0.2, y=0, z=0)))
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='weed',
                                                                   position=rosys.geometry.Point3d(x=0.3, y=-0.08, z=0)))
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='maize',
                                                                   position=rosys.geometry.Point3d(x=0.3, y=0, z=0)))
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='weed',
                                                                   position=rosys.geometry.Point3d(x=0.34, y=0, z=0)))
    assert len(detector.simulated_objects) == 5
    system.current_implement = system.implements['Weed Screw']
    system.current_navigation = system.straight_line_navigation
    system.current_navigation.length = 1.5
    assert isinstance(system.current_implement, WeedingScrew)
    system.current_implement.cultivated_crop = 'maize'
    system.current_implement.max_crop_distance = 0.050
    system.automator.start()
    await forward(50)
    assert system.odometer.prediction.point.x == pytest.approx(system.straight_line_navigation.length, abs=0.1)
    assert len(detector.simulated_objects) == 4, 'last weed should be removed'


@pytest.mark.parametrize('system', ['rb28'], indirect=True)
async def test_tornado_removes_weeds_around_crop(system: System, detector: rosys.vision.DetectorSimulation):
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='sugar_beet',
                                                                   position=rosys.geometry.Point3d(x=0.1, y=0.0, z=0)))
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='weed',
                                                                   position=rosys.geometry.Point3d(x=0.13, y=0.0, z=0)))
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='weed',
                                                                   position=rosys.geometry.Point3d(x=0.1, y=0.05, z=0)))
    system.current_implement = system.implements['Tornado']
    system.current_navigation = system.straight_line_navigation
    system.automator.start()
    await forward(20)
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
