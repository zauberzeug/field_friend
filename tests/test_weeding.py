import pytest
import rosys
from rosys.geometry import Pose
from rosys.testing import forward

from field_friend import System
from field_friend.automations.implements import Tornado, WeedingScrew
from field_friend.automations.navigation import DriveSegment, StraightLineNavigation


async def test_working_with_weeding_screw(system: System, detector: rosys.vision.DetectorSimulation):
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='maize',
                                                                   position=rosys.geometry.Point3d(x=0.2, y=0.0, z=0)))
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='weed',
                                                                   position=rosys.geometry.Point3d(x=0.2, y=0.05, z=0)))
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='weed',
                                                                   position=rosys.geometry.Point3d(x=0.2, y=-0.08, z=0)))
    assert isinstance(system.current_navigation, StraightLineNavigation)
    system.current_implement = system.implements['Weed Screw']
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
    assert isinstance(system.current_navigation, StraightLineNavigation)
    system.current_implement = system.implements['Weed Screw']
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: system.automator.is_stopped)
    assert len(detector.simulated_objects) == 1
    assert detector.simulated_objects[0].category_name == 'maize'


async def test_weeding_screw_crop_distance(system: System, detector: rosys.vision.DetectorSimulation):
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='maize',
                                                                   position=rosys.geometry.Point3d(x=0.2, y=0.0, z=0)))
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='weed',
                                                                   position=rosys.geometry.Point3d(x=0.24, y=0, z=0)))
    # will be skipped because it is too far from the crop
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='weed',
                                                                   position=rosys.geometry.Point3d(x=0.5, y=0, z=0)))

    assert isinstance(system.current_navigation, StraightLineNavigation)
    system.current_implement = system.implements['Weed Screw']
    assert isinstance(system.current_implement, WeedingScrew)
    system.current_implement.cultivated_crop = 'maize'
    system.current_implement.max_crop_distance = 0.05
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: system.automator.is_stopped)
    assert len(detector.simulated_objects) == 2
    assert detector.simulated_objects[1].category_name == 'weed'
    assert detector.simulated_objects[1].position.x == 0.5


async def test_implement_usage(system: System, detector: rosys.vision.DetectorSimulation):
    keep_weeds = [
        rosys.vision.SimulatedObject(category_name='weed', position=rosys.geometry.Point3d(x=0.5, y=0, z=0)),
        rosys.vision.SimulatedObject(category_name='weed', position=rosys.geometry.Point3d(x=2.05, y=0, z=0)),
        rosys.vision.SimulatedObject(category_name='weed', position=rosys.geometry.Point3d(x=2.5, y=0, z=0)),
        rosys.vision.SimulatedObject(category_name='weed', position=rosys.geometry.Point3d(x=3.1, y=0, z=0))
    ]
    destroy_weeds = [
        rosys.vision.SimulatedObject(category_name='weed', position=rosys.geometry.Point3d(x=1.4, y=0, z=0)),
        # TODO: targets directly on the end of a segment are not weeded. Will be fixed with https://github.com/zauberzeug/field_friend/pull/352
        # rosys.vision.SimulatedObject(category_name='weed', position=rosys.geometry.Point3d(x=1.5, y=0, z=0)),
        rosys.vision.SimulatedObject(category_name='weed', position=rosys.geometry.Point3d(x=1.9, y=0, z=0))
    ]
    detector.simulated_objects.extend(keep_weeds)
    detector.simulated_objects.extend(destroy_weeds)
    assert isinstance(system.current_navigation, StraightLineNavigation)
    system.current_implement = system.implements['Weed Screw']
    assert isinstance(system.current_implement, WeedingScrew)

    def generate_path():
        return [
            DriveSegment.from_poses(Pose(x=1.0, y=0.0, yaw=0.0), Pose(x=1.5, y=0.0, yaw=0.0),
                                    use_implement=True, stop_at_end=False),
            DriveSegment.from_poses(Pose(x=1.5, y=0.0, yaw=0.0), Pose(x=2.0, y=0.0, yaw=0.0),
                                    use_implement=True, stop_at_end=False),
            DriveSegment.from_poses(Pose(x=2.0, y=0.0, yaw=0.0), Pose(x=3.0, y=0.0, yaw=0.0), use_implement=False)
        ]
    system.current_navigation.generate_path = generate_path  # type: ignore[assignment]
    system.current_navigation.linear_speed_limit = 0.13
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: system.automator.is_stopped)
    assert len(detector.simulated_objects) == len(keep_weeds)


@pytest.mark.parametrize('work_x', (0.0, 0.3))
async def test_work_offset_navigation(system: System, detector: rosys.vision.DetectorSimulation, work_x: float):
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='weed',
                                                                   position=rosys.geometry.Point3d(x=0.5, y=0.0, z=0.0)))
    assert isinstance(system.current_navigation, StraightLineNavigation)
    system.current_implement = system.implements['Weed Screw']
    system.field_friend.WORK_X = work_x
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: system.automator.is_stopped)
    assert system.robot_locator.pose.point.x == pytest.approx(system.current_navigation.length, abs=0.1)
    assert len(detector.simulated_objects) == 0


async def test_advance_when_target_behind_robot(system: System, detector: rosys.vision.DetectorSimulation):
    # will be weeded normally
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='weed',
                                                                   position=rosys.geometry.Point3d(x=0.812, y=-0.008, z=0)))
    # will be weeded although it is behind the robot, because it is very close to the robot
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='weed',
                                                                   position=rosys.geometry.Point3d(x=system.field_friend.WORK_X-0.004, y=0.1, z=0)))
    # will be skipped because it is too far behind the robot
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='weed',
                                                                   position=rosys.geometry.Point3d(x=system.field_friend.WORK_X-0.006, y=-0.1, z=0)))
    assert isinstance(system.current_navigation, StraightLineNavigation)
    system.current_implement = system.implements['Weed Screw']
    assert isinstance(system.current_navigation.implement, WeedingScrew)
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: system.automator.is_stopped, timeout=1000)
    assert len(detector.simulated_objects) == 1


@pytest.mark.parametrize('system', ['u4'], indirect=True)
async def test_tornado_removes_weeds_around_crop(system: System, detector: rosys.vision.DetectorSimulation):
    assert isinstance(system.implements['Tornado'], Tornado)
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
        rosys.vision.SimulatedObject(category_name='weed',
                                     position=rosys.geometry.Point3d(x=0.2, y=INNER_RADIUS + 0.01, z=0)),
        rosys.vision.SimulatedObject(category_name='weed',
                                     position=rosys.geometry.Point3d(x=0.2, y=OUTER_RADIUS - 0.01, z=0))
    ]
    detector.simulated_objects.extend(targets)
    assert isinstance(system.current_navigation, StraightLineNavigation)
    system.current_implement = system.implements['Tornado']
    assert isinstance(system.current_navigation.implement, Tornado)
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
        rosys.vision.SimulatedObject(
            category_name='weed', position=rosys.geometry.Point3d(x=0.4, y=MIN_RADIUS + 0.01, z=0)),
        rosys.vision.SimulatedObject(
            category_name='weed',  position=rosys.geometry.Point3d(x=0.4, y=MAX_RADIUS - 0.01, z=0))
    ]
    detector.simulated_objects.extend(targets)
    assert isinstance(system.current_navigation, StraightLineNavigation)
    system.current_implement = system.implements['Tornado']
    assert isinstance(system.current_implement, Tornado)
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
    assert isinstance(system.current_navigation, StraightLineNavigation)
    system.current_implement = system.implements['Tornado']
    assert isinstance(system.current_implement, Tornado)
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
    assert isinstance(system.current_navigation, StraightLineNavigation)
    system.current_implement = system.implements['Tornado']
    assert isinstance(system.current_implement, Tornado)
    system.current_implement.drill_between_crops = True
    system.automator.start(system.current_navigation.start())
    await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: system.automator.is_stopped)
    assert len(detector.simulated_objects) == 2
    assert detector.simulated_objects[0].category_name == 'sugar_beet'
    assert detector.simulated_objects[1].category_name == 'sugar_beet'
