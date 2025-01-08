import math
import random

import numpy as np
import pytest
import rosys
from conftest import ROBOT_GEO_START_POSITION
from rosys.testing import assert_point, forward

from field_friend import System
from field_friend.automations import Field
from field_friend.automations.implements import Implement, Recorder, Tornado, WeedingImplement
from field_friend.automations.navigation import StraightLineNavigation
from field_friend.automations.navigation.field_navigation import State as FieldNavigationState


async def test_straight_line(system: System):
    assert_point(system.robot_locator.pose.point, rosys.geometry.Point(x=0.0, y=0))
    assert isinstance(system.current_navigation, StraightLineNavigation)
    assert isinstance(system.current_navigation.implement, Recorder)
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: system.automator.is_stopped)
    assert not system.automator.is_running, 'automation should stop after default length'
    assert system.robot_locator.pose.point.x == pytest.approx(system.straight_line_navigation.length, abs=0.1)


async def test_straight_line_with_tornado(system_with_tornado: System):
    system = system_with_tornado
    assert_point(system.robot_locator.pose.point, rosys.geometry.Point(x=0.0, y=0))
    assert isinstance(system.current_navigation, StraightLineNavigation)
    assert isinstance(system.current_navigation.implement, Recorder)
    system.current_implement = system.implements['Tornado']
    assert isinstance(system.current_navigation.implement, Tornado)
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: system.automator.is_stopped)
    assert not system.automator.is_running, 'automation should stop after default length'
    assert system.robot_locator.pose.point.x == pytest.approx(system.straight_line_navigation.length, abs=0.1)
    assert system.automator.is_stopped


async def test_straight_line_with_high_angles(system: System):
    assert isinstance(system.field_friend.wheels, rosys.hardware.WheelsSimulation)
    predicted_yaw = 190
    start_yaw = system.robot_locator.pose.yaw
    target_yaw = start_yaw + np.deg2rad(predicted_yaw)
    await system.driver.wheels.drive(*system.driver._throttle(0, 0.1))  # pylint: disable=protected-access
    await forward(until=lambda: abs(rosys.helpers.angle(system.robot_locator.pose.yaw, target_yaw)) < np.deg2rad(0.1))
    await system.driver.wheels.stop()
    assert isinstance(system.current_navigation, StraightLineNavigation)
    system.current_navigation.length = 1.0
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: system.automator.is_stopped)
    assert system.robot_locator.pose.point.x == pytest.approx(-0.985, abs=0.1)
    assert system.robot_locator.pose.point.y == pytest.approx(-0.174, abs=0.1)
    assert system.robot_locator.pose.yaw_deg == pytest.approx(predicted_yaw, abs=5)


async def test_driving_to_exact_positions(system: System):
    class Stopper(Implement):
        def __init__(self, system: System) -> None:
            super().__init__('Stopper')
            self.system = system
            self.current_stretch = 0.0
            self.workflow_started = False

        async def get_stretch(self, max_distance: float) -> float:
            self.current_stretch = random.uniform(0.02, max_distance)
            return self.current_stretch

        async def start_workflow(self) -> None:
            self.workflow_started = True
            deadline = rosys.time() + 1
            while self.workflow_started and rosys.time() < deadline:
                await rosys.sleep(0.1)
            self.workflow_started = False

    system.current_implement = stopper = Stopper(system)
    assert isinstance(system.current_navigation, StraightLineNavigation)
    system.current_navigation.linear_speed_limit = 0.02  # drive really slow so we can archive the accuracy tested below
    system.automator.start()

    await forward(until=lambda: system.automator.is_running, dt=0.01)
    for _ in range(20):
        old_position = system.robot_locator.pose.point
        await forward(until=lambda: stopper.workflow_started and system.automator.is_running, dt=0.01)
        distance = old_position.distance(system.robot_locator.pose.point)
        assert distance == pytest.approx(stopper.current_stretch, abs=0.001)
        stopper.workflow_started = False
        await forward(0.1)  # give robot time to update position


async def test_driving_straight_line_with_slippage(system: System):
    assert isinstance(system.field_friend.wheels, rosys.hardware.WheelsSimulation)
    assert isinstance(system.current_navigation, StraightLineNavigation)
    system.current_navigation.length = 2.0
    system.field_friend.wheels.slip_factor_right = 0.0
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: system.automator.is_stopped)
    assert_point(system.robot_locator.pose.point, rosys.geometry.Point(x=2.0, y=0))


async def test_follow_crops_no_direction(system: System, detector: rosys.vision.DetectorSimulation):
    for i in range(1, 3):
        x = i*0.4
        y = i/10
        p = rosys.geometry.Point3d(x=x, y=y, z=0)
        detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='maize', position=p))
    system.current_navigation = system.follow_crops_navigation
    assert isinstance(system.current_navigation.implement, Recorder)
    system.automator.start()
    await forward(2)
    assert system.automator.is_running
    await forward(until=lambda: not system.automator.is_running, timeout=300)
    assert not system.automator.is_running, 'automation should stop if no crops are detected anymore'
    assert system.robot_locator.pose.distance(rosys.geometry.Point3d(x=0, y=0, z=0)) == pytest.approx(2.0, abs=0.1)
    assert system.robot_locator.pose.point.x == pytest.approx(2.0, abs=0.1)
    assert system.robot_locator.pose.point.y == pytest.approx(0, abs=0.01)
    assert system.robot_locator.pose.yaw_deg == pytest.approx(0, abs=1.0)


async def test_follow_crops_empty(system: System, detector: rosys.vision.DetectorSimulation):
    system.current_navigation = system.follow_crops_navigation
    assert isinstance(system.current_navigation.implement, Recorder)
    system.automator.start()
    await forward(2)
    assert system.automator.is_running
    await forward(until=lambda: not system.automator.is_running, timeout=300)
    assert not system.automator.is_running, 'automation should stop if no crops are detected anymore'
    assert system.robot_locator.pose.point.x == pytest.approx(2.0, abs=0.1)
    assert system.robot_locator.pose.point.y == pytest.approx(0, abs=0.01)
    assert system.robot_locator.pose.yaw_deg == pytest.approx(0, abs=1.0)


async def test_follow_crops_straight(system: System, detector: rosys.vision.DetectorSimulation):
    for i in range(10):
        x = i/10
        p = rosys.geometry.Point3d(x=x)
        detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='maize', position=p))
    system.current_navigation = system.follow_crops_navigation
    assert isinstance(system.current_navigation.implement, Recorder)
    system.automator.start()
    await forward(2)
    assert system.automator.is_running
    await forward(until=lambda: not system.automator.is_running, timeout=300)
    assert not system.automator.is_running, 'automation should stop if no crops are detected anymore'
    assert system.robot_locator.pose.point.x == pytest.approx(2.0, abs=0.1)
    assert system.robot_locator.pose.point.y == pytest.approx(0, abs=0.1)
    assert system.robot_locator.pose.yaw_deg == pytest.approx(0, abs=1.0)


async def test_follow_crops_continue(system: System, detector: rosys.vision.DetectorSimulation):
    for i in range(0, 20):
        x = i / 10
        p = rosys.geometry.Point3d(x=x + 1, y=x, z=0)
        detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='maize', position=p))
    system.current_navigation = system.follow_crops_navigation
    system.current_navigation.length = 5.0
    assert isinstance(system.current_navigation.implement, Recorder)
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: not system.automator.is_running, timeout=300)
    assert not system.automator.is_running, 'automation should stop if no crops are detected anymore'
    assert system.robot_locator.pose.point.x == pytest.approx(4.0, abs=0.1)
    assert system.robot_locator.pose.point.y == pytest.approx(3.0, abs=0.1)
    assert system.robot_locator.pose.yaw_deg == pytest.approx(45, abs=1.0)


async def test_follow_crops_adjust(system: System, detector: rosys.vision.DetectorSimulation):
    for i in range(1, 51):
        x = i*0.4
        y = -(i/20)
        p = rosys.geometry.Point3d(x=x, y=y, z=0)
        detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='maize', position=p))
    system.current_navigation = system.follow_crops_navigation
    system.current_navigation.length = 10.0
    assert isinstance(system.current_navigation.implement, Recorder)
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: not system.automator.is_running, timeout=300)
    assert not system.automator.is_running, 'automation should stop if no crops are detected anymore'
    assert system.robot_locator.pose.point.x == pytest.approx(9.93, abs=0.1)
    assert system.robot_locator.pose.point.y == pytest.approx(-1.24, abs=0.1)
    assert system.robot_locator.pose.yaw_deg == pytest.approx(-7.2, abs=1.0)


async def test_follow_crops_curve(system: System, detector: rosys.vision.DetectorSimulation):
    end = rosys.geometry.Point(x=0, y=0)
    for i in range(1, 56):
        x = i/10.0
        p = rosys.geometry.Point3d(x=x, y=(x/4) ** 2, z=0)
        p = system.robot_locator.pose.transform3d(p)
        detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='maize', position=p))
        end = p.projection()
    system.current_navigation = system.follow_crops_navigation
    system.current_navigation.length = end.distance(rosys.geometry.Point(x=0, y=0))
    assert isinstance(system.current_navigation.implement, Recorder)
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: system.robot_locator.pose.distance(end) < 0.2)
    assert system.robot_locator.pose.yaw_deg == pytest.approx(34, abs=5.0)


async def test_follow_crops_outlier(system: System, detector: rosys.vision.DetectorSimulation):
    for i in range(21):
        x = i/10
        y = 0.2 if i == 5 else 0
        p = rosys.geometry.Point3d(x=x, y=y, z=0)
        detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='maize', position=p))
    system.current_navigation = system.follow_crops_navigation
    system.current_navigation.length = 2.6
    assert isinstance(system.current_navigation.implement, Recorder)
    system.automator.start()
    await forward(2)
    assert system.automator.is_running
    await forward(until=lambda: not system.automator.is_running, timeout=300)
    assert not system.automator.is_running, 'automation should stop if no crops are detected anymore'
    assert system.robot_locator.pose.point.x == pytest.approx(2.6, abs=0.1)
    assert system.robot_locator.pose.point.y == pytest.approx(0, abs=0.05)
    assert system.robot_locator.pose.yaw_deg == pytest.approx(0, abs=2)


async def test_follow_crops_outlier_last(system: System, detector: rosys.vision.DetectorSimulation):
    for i in range(20):
        x = i/10
        y = 0
        p = rosys.geometry.Point3d(x=x, y=y, z=0)
        detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='maize', position=p))
    outlier = rosys.geometry.Point3d(x=2.1, y=-0.2, z=0)
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='maize', position=outlier))
    system.current_navigation = system.follow_crops_navigation
    system.current_navigation.length = 2.6
    assert isinstance(system.current_navigation.implement, Recorder)
    system.automator.start()
    await forward(2)
    assert system.automator.is_running
    await forward(until=lambda: not system.automator.is_running, timeout=300)
    assert not system.automator.is_running, 'automation should stop if no crops are detected anymore'
    assert system.robot_locator.pose.point.x == pytest.approx(2.6, abs=0.1)
    assert 0 >= system.robot_locator.pose.point.y >= -0.25
    assert 0 >= system.robot_locator.pose.yaw_deg >= -45


async def test_follow_crops_with_slippage(system: System, detector: rosys.vision.DetectorSimulation):
    for i in range(20):
        x = i/10.0
        p = rosys.geometry.Point3d(x=x, y=(x/3) ** 3, z=0)
        p = system.robot_locator.pose.transform3d(p)
        detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='maize', position=p))
    system.current_navigation = system.follow_crops_navigation
    assert isinstance(system.field_friend.wheels, rosys.hardware.WheelsSimulation)
    system.field_friend.wheels.slip_factor_right = 0.05
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: system.automator.is_stopped)
    assert system.robot_locator.pose.yaw_deg == pytest.approx(16.5, abs=1.0)
    await forward(5)  # waiting for the kalman filter to catch up with gnss updates
    assert system.robot_locator.pose.yaw_deg == pytest.approx(16.5, abs=0.5)


async def test_approach_first_row(system: System, field: Field):
    # pylint: disable=protected-access
    assert system.gnss.last_measurement
    assert ROBOT_GEO_START_POSITION is not None
    assert system.gnss.last_measurement.point.distance(ROBOT_GEO_START_POSITION) < 0.01
    system.field_navigation.field_id = field.id
    system.current_navigation = system.field_navigation
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: system.field_navigation.automation_watcher.field_watch_active)
    await forward(until=lambda: system.current_implement.is_active)
    await forward(until=lambda: system.field_navigation._state == FieldNavigationState.APPROACH_START_ROW)
    for index, point in enumerate(field.rows[0].points):
        assert system.field_navigation.current_row.points[index].distance(point) == pytest.approx(0, abs=1e-8)
    assert system.field_navigation.automation_watcher.field_watch_active
    assert system.automator.is_running
    await forward(until=lambda: system.field_navigation._state == FieldNavigationState.FOLLOW_ROW)
    start_point = field.rows[0].points[0].to_local()
    assert system.robot_locator.pose.point.x == pytest.approx(start_point.x, abs=0.05)
    assert system.robot_locator.pose.point.y == pytest.approx(start_point.y, abs=0.05)
    assert system.field_navigation.automation_watcher.field_watch_active


async def test_approach_first_row_from_other_side(system: System, field: Field):
    # pylint: disable=protected-access
    assert system.gnss.last_measurement
    assert ROBOT_GEO_START_POSITION is not None
    assert system.gnss.last_measurement.point.distance(ROBOT_GEO_START_POSITION) < 0.01
    start_point = field.rows[0].points[-1].to_local()
    end_point = field.rows[0].points[0].to_local()
    safe_start_point = start_point.polar(-1.0, start_point.direction(end_point))

    async def drive_to_start():
        await system.driver.drive_to(safe_start_point)
    system.automator.start(drive_to_start())
    await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: system.automator.is_stopped)
    assert_point(system.robot_locator.pose.point, safe_start_point, tolerance=0.05)
    system.field_navigation.field_id = field.id
    system.current_navigation = system.field_navigation
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: system.field_navigation.automation_watcher.field_watch_active)
    await forward(until=lambda: system.current_implement.is_active)
    await forward(until=lambda: system.field_navigation._state == FieldNavigationState.APPROACH_START_ROW)
    for index, point in enumerate(field.rows[0].points):
        assert system.field_navigation.current_row.points[index].distance(point) == pytest.approx(0, abs=1e-8)
    assert system.field_navigation.automation_watcher.field_watch_active
    assert system.automator.is_running
    await forward(until=lambda: system.field_navigation._state == FieldNavigationState.FOLLOW_ROW)
    assert system.robot_locator.pose.point.x == pytest.approx(start_point.x, abs=0.05)
    assert system.robot_locator.pose.point.y == pytest.approx(start_point.y, abs=0.05)
    assert system.field_navigation.automation_watcher.field_watch_active


async def test_approach_first_row_when_outside_of_field(system: System, field: Field):
    # pylint: disable=protected-access
    assert system.gnss.last_measurement
    assert ROBOT_GEO_START_POSITION is not None
    assert system.gnss.last_measurement.point.distance(ROBOT_GEO_START_POSITION) < 0.01
    point_outside = rosys.geometry.Point(x=-10, y=0)

    async def drive_away():
        await system.driver.drive_to(point_outside)
    system.automator.start(drive_away())
    await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: system.automator.is_stopped)
    assert_point(system.robot_locator.pose.point, point_outside, tolerance=0.05)
    system.field_navigation.field_id = field.id
    system.current_navigation = system.field_navigation
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: system.field_navigation.automation_watcher.field_watch_active)
    await forward(until=lambda: system.current_implement.is_active)
    for index, point in enumerate(field.rows[0].points):
        assert system.field_navigation.current_row.points[index].distance(point) == pytest.approx(0, abs=1e-8)
    await forward(until=lambda: system.automator.is_stopped)
    assert system.field_navigation._state == FieldNavigationState.APPROACH_START_ROW
    assert system.robot_locator.pose.point.x == pytest.approx(-10, abs=1.0)
    assert system.robot_locator.pose.point.y == pytest.approx(0.0, abs=0.5)
    assert not system.automator.is_running, 'should have been stopped because robot is outside of field boundaries'


async def test_complete_row(system: System, field: Field):
    # pylint: disable=protected-access
    assert system.gnss.last_measurement
    assert ROBOT_GEO_START_POSITION is not None
    assert system.gnss.last_measurement.point.distance(ROBOT_GEO_START_POSITION) < 0.01
    system.field_navigation.field_id = field.id
    system.current_navigation = system.field_navigation
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: system.field_navigation.automation_watcher.field_watch_active)
    await forward(until=lambda: system.field_navigation._state == FieldNavigationState.APPROACH_START_ROW)
    for index, point in enumerate(field.rows[0].points):
        assert system.field_navigation.current_row.points[index].distance(point) == pytest.approx(0, abs=1e-8)
    assert system.field_navigation.automation_watcher.field_watch_active
    await forward(until=lambda: system.field_navigation._state == FieldNavigationState.FOLLOW_ROW)
    await forward(until=lambda: system.field_navigation._state == FieldNavigationState.CHANGE_ROW)
    end_point = field.rows[0].points[1].to_local()
    assert_point(system.robot_locator.pose.point, end_point, tolerance=0.05)
    assert system.field_navigation.automation_watcher.field_watch_active


async def test_resuming_field_navigation_after_automation_stop(system: System, field: Field):
    # pylint: disable=protected-access
    assert system.gnss.last_measurement
    assert ROBOT_GEO_START_POSITION is not None
    assert system.gnss.last_measurement.point.distance(ROBOT_GEO_START_POSITION) < 0.01
    system.field_provider.select_field(field.id)
    system.current_navigation = system.field_navigation
    system.automator.start()
    await forward(until=lambda: system.field_navigation._state == FieldNavigationState.FOLLOW_ROW)
    point = rosys.geometry.Point(x=0.3, y=0.0)
    await forward(x=point.x, y=point.y, tolerance=0.05)
    system.automator.stop(because='test')
    system.automator.start()
    await forward(until=lambda: system.field_navigation._state == FieldNavigationState.FOLLOW_ROW)
    current_row = system.field_navigation.current_row
    assert system.field_navigation.start_point == current_row.points[0].to_local()
    assert system.field_navigation.end_point == current_row.points[1].to_local()
    await forward(until=lambda: system.field_navigation._state == FieldNavigationState.CHANGE_ROW)
    await forward(until=lambda: system.field_navigation._state == FieldNavigationState.FOLLOW_ROW)
    current_row = system.field_navigation.current_row
    assert system.field_navigation.start_point == current_row.points[1].to_local()
    assert system.field_navigation.end_point == current_row.points[0].to_local()
    assert field.rows[1].id == system.field_navigation.current_row.id
    await forward(until=lambda: system.field_navigation._state == FieldNavigationState.FIELD_COMPLETED, timeout=1500)
    await forward(until=lambda: system.automator.is_stopped)
    end_point = field.rows[-1].points[0].to_local()
    assert system.robot_locator.pose.point.distance(end_point) < 0.1


@pytest.mark.parametrize('offset', (0, -0.06))
async def test_field_navigation_robot_between_rows(system: System, field: Field, offset: float):
    assert system.gnss.last_measurement
    assert ROBOT_GEO_START_POSITION is not None
    assert system.gnss.last_measurement.point.distance(ROBOT_GEO_START_POSITION) < 0.01

    row_start = field.rows[0].points[0].to_local()
    row_end = field.rows[0].points[1].to_local()
    row_direction = row_start.direction(row_end)
    offset_direction = row_direction + math.pi/2
    offset_point = row_start.polar(0.5, row_direction)
    if offset > 0:
        offset_point = offset_point.polar(offset, offset_direction)

    async def drive_to_offset():
        await system.driver.drive_to(offset_point)
        target_yaw = offset_point.direction(row_end)
        await system.field_navigation.turn_to_yaw(target_yaw)
    system.automator.start(drive_to_offset())
    await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: system.automator.is_stopped)
    assert system.robot_locator.pose.point.distance(offset_point) < 0.01

    system.field_navigation.field_id = field.id
    system.current_navigation = system.field_navigation
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: system.automator.is_stopped, timeout=1500)
    end_point = field.rows[-1].points[0].to_local()
    if offset > system.field_navigation.MAX_DISTANCE_DEVIATION:
        assert system.robot_locator.pose.point.x != pytest.approx(end_point.x, abs=0.05)
        assert system.robot_locator.pose.point.y != pytest.approx(end_point.y, abs=0.05)
        assert_point(system.robot_locator.pose.point, offset_point, tolerance=0.05)
    else:
        assert_point(system.robot_locator.pose.point, end_point, tolerance=0.05)


@pytest.mark.parametrize('heading_degrees', (0, 40))
async def test_field_navigation_robot_heading_deviation(system: System, field: Field, heading_degrees: float):
    assert system.gnss.last_measurement
    assert ROBOT_GEO_START_POSITION is not None
    assert system.gnss.last_measurement.point.distance(ROBOT_GEO_START_POSITION) < 0.01

    row_start = field.rows[0].points[0].to_local()
    row_end = field.rows[0].points[1].to_local()
    row_direction = row_start.direction(row_end)
    offset_point = row_start.polar(0.5, row_direction)

    async def drive_to_offset():
        await system.driver.drive_to(offset_point)
        target_yaw = offset_point.direction(row_end) + np.deg2rad(heading_degrees)
        await system.field_navigation.turn_to_yaw(target_yaw)
    system.automator.start(drive_to_offset())
    await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: system.automator.is_stopped)
    assert system.robot_locator.pose.point.distance(offset_point) < 0.01

    system.field_navigation.field_id = field.id
    system.current_navigation = system.field_navigation
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: system.automator.is_stopped, timeout=1500)
    end_point = field.rows[-1].points[0].to_local()
    # TODO: robot_in_working_area
    if np.deg2rad(heading_degrees) > system.field_navigation.MAX_ANGLE_DEVIATION:
        assert system.odometer.prediction.point.x != pytest.approx(end_point.x, abs=0.05)
        assert system.odometer.prediction.point.y != pytest.approx(end_point.y, abs=0.05)
        assert_point(system.robot_locator.pose.point, offset_point, tolerance=0.05)
    else:
        assert_point(system.robot_locator.pose.point, end_point, tolerance=0.05)


async def test_complete_field(system: System, field: Field):
    # pylint: disable=protected-access
    assert system.gnss.last_measurement
    assert ROBOT_GEO_START_POSITION is not None
    assert system.gnss.last_measurement.point.distance(ROBOT_GEO_START_POSITION) < 0.01
    system.field_navigation.field_id = field.id
    system.current_navigation = system.field_navigation
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: system.field_navigation._state == FieldNavigationState.FIELD_COMPLETED, timeout=1500)
    assert system.automator.is_stopped
    end_point = field.rows[-1].points[0].to_local()
    assert_point(system.robot_locator.pose.point, end_point, tolerance=0.05)


async def test_complete_field_with_selected_beds(system: System, field_with_beds: Field):
    # pylint: disable=protected-access
    assert system.gnss.last_measurement
    assert ROBOT_GEO_START_POSITION is not None
    assert system.gnss.last_measurement.point.distance(ROBOT_GEO_START_POSITION) < 0.01
    system.field_provider.select_field(field_with_beds.id)
    system.field_provider._only_specific_beds = True
    system.field_provider.selected_beds = [1, 3]
    system.current_navigation = system.field_navigation
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: system.field_navigation._state == FieldNavigationState.FIELD_COMPLETED, timeout=1500)
    assert system.automator.is_stopped
    end_point = field_with_beds.rows[2].points[0].to_local()
    assert_point(system.robot_locator.pose.point, end_point, tolerance=0.05)


async def test_complete_field_without_second_bed(system: System, field_with_beds: Field):
    # pylint: disable=protected-access
    assert system.gnss.last_measurement
    assert ROBOT_GEO_START_POSITION is not None
    assert system.gnss.last_measurement.point.distance(ROBOT_GEO_START_POSITION) < 0.01
    system.field_provider.select_field(field_with_beds.id)
    system.field_provider._only_specific_beds = True
    system.field_provider.selected_beds = [1, 3, 4]
    system.current_navigation = system.field_navigation
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: system.field_navigation._state == FieldNavigationState.FIELD_COMPLETED, timeout=1500)
    assert system.automator.is_stopped
    end_point = field_with_beds.rows[-1].points[1].to_local()
    assert_point(system.robot_locator.pose.point, end_point, tolerance=0.05)


async def test_field_with_first_row_excluded(system: System, field_with_beds: Field):
    assert system.gnss.last_measurement
    assert ROBOT_GEO_START_POSITION is not None
    assert system.gnss.last_measurement.point.distance(ROBOT_GEO_START_POSITION) < 0.01

    system.field_provider.select_field(field_with_beds.id)
    system.field_provider._only_specific_beds = True
    system.field_provider.selected_beds = [2, 3, 4]  # Exclude bed 1

    system.current_navigation = system.field_navigation
    system.automator.start()
    await forward(until=lambda: not system.automator.is_stopped)
    assert system.robot_locator.pose.point.x == pytest.approx(0, abs=0.01)
    assert system.robot_locator.pose.point.y == pytest.approx(0, abs=0.01)
    assert system.robot_locator.pose.yaw_deg == pytest.approx(0, abs=0.1)


async def test_field_with_bed_crops(system: System, field_with_beds: Field):
    # pylint: disable=protected-access
    assert system.gnss.last_measurement
    assert ROBOT_GEO_START_POSITION is not None
    assert system.gnss.last_measurement.point.distance(ROBOT_GEO_START_POSITION) < 0.01

    system.field_provider.select_field(field_with_beds.id)
    system.current_navigation = system.field_navigation
    system.current_implement = system.implements['Weed Screw']
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    assert isinstance(system.current_implement, WeedingImplement)
    for bed_number in range(field_with_beds.bed_count):
        await forward(until=lambda: system.field_navigation._state == FieldNavigationState.FOLLOW_ROW)
        expected_crop = field_with_beds.bed_crops[str(bed_number)]
        assert system.current_implement.cultivated_crop == expected_crop
        if bed_number != field_with_beds.bed_count - 1:
            await forward(until=lambda: system.field_navigation._state == FieldNavigationState.CHANGE_ROW)
    await forward(until=lambda: system.field_navigation._state == FieldNavigationState.FIELD_COMPLETED, timeout=1500)
    await forward(until=lambda: system.automator.is_stopped)
    end_point = field_with_beds.rows[-1].points[0].to_local()
    assert_point(system.robot_locator.pose.point, end_point, tolerance=0.05)
    assert system.automator.is_stopped


async def test_field_with_bed_crops_with_tornado(system_with_tornado: System, field_with_beds_tornado: Field, detector: rosys.vision.DetectorSimulation):
    # TODO: crop is None
    system = system_with_tornado
    field_with_beds = field_with_beds_tornado
    assert system.gnss.last_measurement
    assert ROBOT_GEO_START_POSITION is not None
    assert system.gnss.last_measurement.point.distance(ROBOT_GEO_START_POSITION) < 0.01

    # bed 1
    detector.simulated_objects.append(rosys.vision.SimulatedObject(
        category_name='sugar_beet', position=rosys.geometry.Point3d(x=0.2, y=0.0, z=0)))
    detector.simulated_objects.append(rosys.vision.SimulatedObject(
        category_name='garlic', position=rosys.geometry.Point3d(x=0.3, y=0.0, z=0)))
    detector.simulated_objects.append(rosys.vision.SimulatedObject(
        category_name='sugar_beet', position=rosys.geometry.Point3d(x=0.4, y=0.0, z=0)))
    # bed 2
    detector.simulated_objects.append(rosys.vision.SimulatedObject(
        category_name='garlic', position=rosys.geometry.Point3d(x=0.2, y=-0.45, z=0)))
    detector.simulated_objects.append(rosys.vision.SimulatedObject(
        category_name='onion', position=rosys.geometry.Point3d(x=0.3, y=-0.45, z=0)))
    detector.simulated_objects.append(rosys.vision.SimulatedObject(
        category_name='garlic', position=rosys.geometry.Point3d(x=0.4, y=-0.45, z=0)))
    # bed 3
    detector.simulated_objects.append(rosys.vision.SimulatedObject(
        category_name='onion', position=rosys.geometry.Point3d(x=0.3, y=-0.9, z=0)))
    detector.simulated_objects.append(rosys.vision.SimulatedObject(
        category_name='lettuce', position=rosys.geometry.Point3d(x=0.3, y=-0.9, z=0)))
    detector.simulated_objects.append(rosys.vision.SimulatedObject(
        category_name='onion', position=rosys.geometry.Point3d(x=0.4, y=-0.9, z=0)))
    # bed4
    detector.simulated_objects.append(rosys.vision.SimulatedObject(
        category_name='lettuce', position=rosys.geometry.Point3d(x=0.2, y=-1.35, z=0)))
    detector.simulated_objects.append(rosys.vision.SimulatedObject(
        category_name='onion', position=rosys.geometry.Point3d(x=0.3, y=-1.35, z=0)))
    detector.simulated_objects.append(rosys.vision.SimulatedObject(
        category_name='lettuce', position=rosys.geometry.Point3d(x=0.4, y=-1.35, z=0)))

    system.field_provider.select_field(field_with_beds.id)
    system.current_navigation = system.field_navigation
    system.current_implement = system.implements['Tornado']
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    assert isinstance(system.current_implement, WeedingImplement)

    for bed_number in range(field_with_beds.bed_count):
        await forward(until=lambda: system.field_navigation._state == FieldNavigationState.FOLLOW_ROW)
        expected_crop = field_with_beds.bed_crops[str(bed_number)]
        assert system.current_implement.cultivated_crop == system.field_navigation.current_row.crop == expected_crop
        current_y = bed_number * -field_with_beds.bed_spacing
        worked_crops = [obj for obj in detector.simulated_objects
                        if obj.category_name == expected_crop
                        and obj.position.y == current_y]
        assert len(worked_crops) == 2, f'Tornado should have worked on 2 {expected_crop} crops in bed {bed_number}'
        wrong_crop = [obj for obj in detector.simulated_objects
                      if obj.category_name != expected_crop
                      and obj.position.y == current_y]
        assert len(wrong_crop) == 1, f'Tornado should have skipped 1 wrong crop in bed {bed_number}'
        if bed_number != field_with_beds.bed_count - 1:
            await forward(until=lambda: system.field_navigation._state == FieldNavigationState.CHANGE_ROW)
    await forward(until=lambda: system.field_navigation._state == FieldNavigationState.FIELD_COMPLETED, timeout=1500)
    await forward(until=lambda: system.automator.is_stopped)
    end_point = field_with_beds.rows[-1].points[0].to_local()
    assert_point(system.robot_locator.pose.point, end_point, tolerance=0.05)
    assert system.automator.is_stopped
