import pytest
import rosys
from rosys.geometry import Pose
from rosys.testing import forward

from field_friend import System
from field_friend.automations.implements import Tornado, WeedingScrew
from field_friend.automations.navigation import PathSegment, WaypointNavigation, WorkingSegment


async def test_working_with_weeding_screw(system: System, detector: rosys.vision.DetectorSimulation):
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='maize',
                                                                   position=rosys.geometry.Point3d(x=0.2, y=0.0, z=0)))
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='weed',
                                                                   position=rosys.geometry.Point3d(x=0.2, y=0.05, z=0)))
    system.current_implement = system.implements['Weed Screw']
    system.current_navigation = system.straight_line_navigation
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: system.automator.is_stopped)
    assert len(detector.simulated_objects) == 1
    assert detector.simulated_objects[0].category_name == 'maize'


async def test_keep_crops_safe(system: System, detector: rosys.vision.DetectorSimulation):
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='maize',
                                                                   position=rosys.geometry.Point3d(x=0.2, y=0.0, z=0)))
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='weed',
                                                                   position=rosys.geometry.Point3d(x=0.2, y=system.field_friend.DRILL_RADIUS-0.01, z=0)))
    system.current_implement = system.implements['Weed Screw']
    system.current_navigation = system.straight_line_navigation
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: system.automator.is_stopped)
    assert len(detector.simulated_objects) == 1
    assert detector.simulated_objects[0].category_name == 'maize'


@pytest.mark.skip(reason='We currently do not differentiate between different weed types')
async def test_weeding_screw_only_targets_big_weed(system: System, detector: rosys.vision.DetectorSimulation):
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='weed',
                                                                   position=rosys.geometry.Point3d(x=0.2, y=0.0, z=0)))
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='coin',
                                                                   position=rosys.geometry.Point3d(x=0.15, y=0, z=0)))
    system.current_implement = system.implements['Weed Screw']
    system.current_navigation = system.straight_line_navigation
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: system.automator.is_stopped)
    assert len(detector.simulated_objects) == 1
    assert detector.simulated_objects[0].category_name == 'weed'


async def test_weeding_screw_does_not_skip_close_weed(system: System, detector: rosys.vision.DetectorSimulation):
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='weed',
                                                                   position=rosys.geometry.Point3d(x=0.2, y=0, z=0)))
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='weed',
                                                                   position=rosys.geometry.Point3d(x=0.2+system.field_friend.DRILL_RADIUS-0.01, y=0.05, z=0)))
    system.current_implement = system.implements['Weed Screw']
    system.current_navigation = system.straight_line_navigation
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: system.automator.is_stopped)
    assert len(detector.simulated_objects) == 0


async def test_weeding_screw_focus_on_weed_close_to_crop(system: System, detector: rosys.vision.DetectorSimulation):
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='maize',
                                                                   position=rosys.geometry.Point3d(x=0.2, y=0.0, z=0)))
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='weed',
                                                                   position=rosys.geometry.Point3d(x=0.1, y=0, z=0)))
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='weed',
                                                                   position=rosys.geometry.Point3d(x=0.16, y=0, z=0)))
    system.current_implement = system.implements['Weed Screw']
    system.current_navigation = system.straight_line_navigation
    assert isinstance(system.current_implement, WeedingScrew)
    system.current_implement.cultivated_crop = 'maize'
    system.current_implement.max_crop_distance = 0.05
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: system.automator.is_stopped)
    assert len(detector.simulated_objects) == 2
    assert detector.simulated_objects[1].category_name == 'weed'
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
    await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: system.automator.is_stopped)
    assert system.robot_locator.pose.point.x == pytest.approx(system.straight_line_navigation.length, abs=0.1)
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
    await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: system.automator.is_stopped)
    assert system.robot_locator.pose.point.x == pytest.approx(system.straight_line_navigation.length, abs=0.1)
    assert len(detector.simulated_objects) == 4, 'last weed should be removed'


async def test_implement_usage(system: System, detector: rosys.vision.DetectorSimulation):
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='weed',
                                                                   position=rosys.geometry.Point3d(x=0.5, y=0.0, z=0.0)))
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='weed',
                                                                   position=rosys.geometry.Point3d(x=1.5, y=0.0, z=0.0)))

    def generate_path():
        pose1 = Pose(x=0.5, y=0.0, yaw=0.0)
        pose2 = Pose(x=0.5, y=0.0, yaw=0.0)
        return [
            WorkingSegment.from_poses(system.robot_locator.pose, pose1, stop_at_end=False),
            PathSegment.from_poses(pose1, pose2),
        ]
    system.current_navigation = system.waypoint_navigation
    system.current_implement = system.implements['Weed Screw']
    assert isinstance(system.current_navigation, WaypointNavigation)
    system.current_navigation.generate_path = generate_path  # type: ignore
    assert len(detector.simulated_objects) == 2
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: system.automator.is_stopped)
    assert len(detector.simulated_objects) == 1


@pytest.mark.parametrize('work_x', (0.0, 0.3))
async def test_work_offset_navigation(system: System, detector: rosys.vision.DetectorSimulation, work_x: float):
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='weed',
                                                                   position=rosys.geometry.Point3d(x=0.5, y=0.0, z=0.0)))
    system.current_implement = system.implements['Weed Screw']
    system.current_navigation = system.straight_line_navigation

    system.field_friend.WORK_X = work_x
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: system.automator.is_stopped)
    assert system.robot_locator.pose.point.x == pytest.approx(system.straight_line_navigation.length, abs=0.1)
    assert len(detector.simulated_objects) == 0


@pytest.mark.parametrize('system', ['u4'], indirect=True)
async def test_tornado_removes_weeds_around_crop(system: System, detector: rosys.vision.DetectorSimulation):
    INNER_DIAMETER, OUTER_DIAMETER = system.field_friend.tornado_diameters(system.implements['Tornado'].tornado_angle)
    INNER_RADIUS = INNER_DIAMETER / 2
    OUTER_RADIUS = OUTER_DIAMETER / 2
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='sugar_beet',
                                                                   position=rosys.geometry.Point3d(x=0.2, y=0.0, z=0)))
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='weed',
                                                                   position=rosys.geometry.Point3d(x=0.2, y=INNER_RADIUS - 0.01, z=0)))
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='weed',
                                                                   position=rosys.geometry.Point3d(x=0.2, y=OUTER_RADIUS + 0.01, z=0)))
    targets = [
        rosys.vision.SimulatedObject(category_name='weed', position=rosys.geometry.Point3d(x=0.2, y=INNER_RADIUS + 0.01, z=0)),
        rosys.vision.SimulatedObject(category_name='weed',  position=rosys.geometry.Point3d(x=0.2, y=OUTER_RADIUS - 0.01, z=0))
    ]
    detector.simulated_objects.extend(targets)
    system.current_implement = system.implements['Tornado']
    system.current_navigation = system.straight_line_navigation
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: system.automator.is_stopped)
    assert len(detector.simulated_objects) == 3
    assert detector.simulated_objects[0].category_name == 'sugar_beet'
    for target in targets:
        assert target not in detector.simulated_objects, f'target {target.position} should be removed'


@pytest.mark.parametrize('system', ['u4'], indirect=True)
async def test_tornado_drill_with_open_tornado(system: System, detector: rosys.vision.DetectorSimulation):
    MIN_RADIUS = 0.069 / 2
    MAX_RADIUS = 0.165 / 2
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='sugar_beet',
                                                                   position=rosys.geometry.Point3d(x=0.4, y=0.0, z=0)))
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='weed',
                                                                   position=rosys.geometry.Point3d(x=0.4, y=MIN_RADIUS - 0.01, z=0)))
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='weed',
                                                                   position=rosys.geometry.Point3d(x=0.4, y=MAX_RADIUS + 0.01, z=0)))
    targets = [
        rosys.vision.SimulatedObject(category_name='weed', position=rosys.geometry.Point3d(x=0.4, y=MIN_RADIUS + 0.01, z=0)),
        rosys.vision.SimulatedObject(category_name='weed',  position=rosys.geometry.Point3d(x=0.4, y=MAX_RADIUS - 0.01, z=0))
    ]
    detector.simulated_objects.extend(targets)
    system.current_implement = system.implements['Tornado']
    system.current_navigation = system.straight_line_navigation
    system.current_implement.tornado_angle = 180
    system.current_implement.drill_with_open_tornado = True
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: system.automator.is_stopped)
    assert len(detector.simulated_objects) == 3
    for target in targets:
        assert target not in detector.simulated_objects, f'target {target.position} should be removed'


@pytest.mark.parametrize('system', ['u4'], indirect=True)
async def test_tornado_skips_crop_if_no_weeds(system: System, detector: rosys.vision.DetectorSimulation):
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='sugar_beet',
                                                                   position=rosys.geometry.Point3d(x=0.2, y=0.0, z=0)))
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='weed',
                                                                   position=rosys.geometry.Point3d(x=0.2, y=0.05, z=0)))
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='sugar_beet',
                                                                   position=rosys.geometry.Point3d(x=0.6, y=0.0, z=0)))
    system.current_implement = system.implements['Tornado']
    system.current_navigation = system.straight_line_navigation
    system.current_implement.skip_if_no_weeds = True
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: system.automator.is_stopped)
    assert len(system.current_implement.last_punches) == 1
    assert len(detector.simulated_objects) == 2


@pytest.mark.skip(reason='Tornado does not yet remove weeds between crops')
@pytest.mark.parametrize('system', ['u4'], indirect=True)
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
    await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: system.automator.is_stopped)
    assert len(detector.simulated_objects) == 2
    assert detector.simulated_objects[0].category_name == 'sugar_beet'
    assert detector.simulated_objects[1].category_name == 'sugar_beet'
