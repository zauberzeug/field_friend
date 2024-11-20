import random

import numpy as np
import pytest
import rosys
from conftest import ROBOT_GEO_START_POSITION
from rosys.testing import forward

from field_friend import System
from field_friend.automations import Field
from field_friend.automations.implements import Implement, Recorder
from field_friend.automations.navigation import StraightLineNavigation
from field_friend.automations.navigation.field_navigation import State as FieldNavigationState
from field_friend.localization import GnssSimulation


async def test_straight_line(system: System):
    assert system.odometer.prediction.point.x == 0
    assert isinstance(system.current_navigation, StraightLineNavigation)
    assert isinstance(system.current_navigation.implement, Recorder)
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: system.automator.is_stopped)
    assert not system.automator.is_running, 'automation should stop after default length'
    assert system.odometer.prediction.point.x == pytest.approx(system.straight_line_navigation.length, abs=0.1)


async def test_straight_line_with_high_angles(system: System):
    assert isinstance(system.field_friend.wheels, rosys.hardware.WheelsSimulation)
    predicted_yaw = 190
    start_yaw = system.odometer.prediction.yaw
    target_yaw = start_yaw + np.deg2rad(predicted_yaw)
    await system.driver.wheels.drive(*system.driver._throttle(0, 0.1))  # pylint: disable=protected-access
    await forward(until=lambda: abs(rosys.helpers.angle(system.odometer.prediction.yaw, target_yaw)) < np.deg2rad(0.01))
    await system.driver.wheels.stop()
    assert isinstance(system.current_navigation, StraightLineNavigation)
    system.current_navigation.length = 1.0
    system.gnss.observed_poses.clear()
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: system.automator.is_stopped)
    assert system.odometer.prediction.point.x == pytest.approx(-0.985, abs=0.1)
    assert system.odometer.prediction.point.y == pytest.approx(-0.174, abs=0.1)
    assert system.odometer.prediction.yaw_deg == pytest.approx(predicted_yaw, abs=5)


async def test_straight_line_with_failing_gnss(system: System, gnss: GnssSimulation, detector: rosys.vision.DetectorSimulation):
    async def empty():
        return None
    create_new_record = gnss._create_new_record
    system.automator.start()
    await forward(5)
    gnss._create_new_record = empty  # type: ignore
    await forward(0.5)
    gnss._create_new_record = create_new_record
    await forward(5)
    assert system.automator.is_running
    assert len(detector.simulated_objects) == 0
    assert system.odometer.prediction.yaw_deg == pytest.approx(0, abs=1)


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
    system.current_navigation.linear_speed_limit = 0.05  # drive really slow so we can archive the accuracy tested below
    system.automator.start()

    await forward(until=lambda: system.automator.is_running, dt=0.01)
    for _ in range(20):
        old_position = system.odometer.prediction.point
        await forward(until=lambda: stopper.workflow_started and system.automator.is_running, dt=0.01)
        distance = old_position.distance(system.odometer.prediction.point)
        assert distance == pytest.approx(stopper.current_stretch, abs=0.001)
        stopper.workflow_started = False
        await forward(0.1)  # give robot time to update position


async def test_driving_straight_line_with_slippage(system: System):
    assert isinstance(system.field_friend.wheels, rosys.hardware.WheelsSimulation)
    assert isinstance(system.current_navigation, StraightLineNavigation)
    system.current_navigation.length = 2.0
    system.field_friend.wheels.slip_factor_right = 0.04
    system.gnss.ensure_gnss = True
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: system.automator.is_stopped)
    assert system.odometer.prediction.point.x == pytest.approx(2.0, abs=0.1)
    assert system.odometer.prediction.point.y == pytest.approx(0.0, abs=0.1)


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
    assert system.odometer.prediction.distance(rosys.geometry.Point3d(x=0, y=0, z=0)) == pytest.approx(2.0, abs=0.1)
    assert system.odometer.prediction.point.x == pytest.approx(2.0, abs=0.1)
    assert system.odometer.prediction.point.y == pytest.approx(0, abs=0.01)
    assert system.odometer.prediction.yaw_deg == pytest.approx(0, abs=1.0)


async def test_follow_crops_empty(system: System, detector: rosys.vision.DetectorSimulation):
    system.current_navigation = system.follow_crops_navigation
    assert isinstance(system.current_navigation.implement, Recorder)
    system.automator.start()
    await forward(2)
    assert system.automator.is_running
    await forward(until=lambda: not system.automator.is_running, timeout=300)
    assert not system.automator.is_running, 'automation should stop if no crops are detected anymore'
    assert system.odometer.prediction.point.x == pytest.approx(2.0, abs=0.1)
    assert system.odometer.prediction.point.y == pytest.approx(0, abs=0.01)
    assert system.odometer.prediction.yaw_deg == pytest.approx(0, abs=1.0)


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
    assert system.odometer.prediction.point.x == pytest.approx(2.0, abs=0.1)
    assert system.odometer.prediction.point.y == pytest.approx(0, abs=0.1)
    assert system.odometer.prediction.yaw_deg == pytest.approx(0, abs=1.0)


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
    assert system.odometer.prediction.point.x == pytest.approx(4.0, abs=0.1)
    assert system.odometer.prediction.point.y == pytest.approx(3.0, abs=0.1)
    assert system.odometer.prediction.yaw_deg == pytest.approx(45, abs=1.0)


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
    assert system.odometer.prediction.point.x == pytest.approx(9.93, abs=0.1)
    assert system.odometer.prediction.point.y == pytest.approx(-1.24, abs=0.1)
    assert system.odometer.prediction.yaw_deg == pytest.approx(-7.2, abs=1.0)


async def test_follow_crops_curve(system: System, detector: rosys.vision.DetectorSimulation):
    end = rosys.geometry.Point(x=0, y=0)
    for i in range(1, 56):
        x = i/10.0
        p = rosys.geometry.Point3d(x=x, y=(x/4) ** 2, z=0)
        p = system.odometer.prediction.transform3d(p)
        detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='maize', position=p))
        end = p.projection()
    system.current_navigation = system.follow_crops_navigation
    system.current_navigation.length = end.distance(rosys.geometry.Point(x=0, y=0))
    assert isinstance(system.current_navigation.implement, Recorder)
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: system.odometer.prediction.distance(end) < 0.2)
    assert system.odometer.prediction.yaw_deg == pytest.approx(34, abs=5.0)


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
    assert system.odometer.prediction.point.x == pytest.approx(2.6, abs=0.1)
    assert system.odometer.prediction.point.y == pytest.approx(0, abs=0.05)
    assert system.odometer.prediction.yaw_deg == pytest.approx(0, abs=2)


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
    assert system.odometer.prediction.point.x == pytest.approx(2.6, abs=0.1)
    assert 0 >= system.odometer.prediction.point.y >= -0.25
    assert 0 >= system.odometer.prediction.yaw_deg >= -45


async def test_follow_crops_with_slippage(system: System, detector: rosys.vision.DetectorSimulation):
    for i in range(20):
        x = i/10.0
        p = rosys.geometry.Point3d(x=x, y=(x/3) ** 3, z=0)
        p = system.odometer.prediction.transform3d(p)
        detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='maize', position=p))
    system.gnss.min_seconds_between_updates = 1
    system.gnss.ensure_gnss = True
    system.current_navigation = system.follow_crops_navigation
    assert isinstance(system.field_friend.wheels, rosys.hardware.WheelsSimulation)
    system.field_friend.wheels.slip_factor_right = 0.05
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: system.automator.is_stopped)
    assert system.odometer.prediction.yaw_deg == pytest.approx(16.5, abs=0.2)


async def test_approaching_first_row(system: System, field: Field):
    # pylint: disable=protected-access
    assert system.gnss.current
    assert system.gnss.current.location.distance(ROBOT_GEO_START_POSITION) < 0.01
    system.field_navigation.field_id = field.id
    system.current_navigation = system.field_navigation
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: system.field_navigation.automation_watcher.field_watch_active)
    await forward(until=lambda: system.current_implement.is_active)
    await forward(until=lambda: system.field_navigation._state == FieldNavigationState.APPROACHING_ROW_START)
    for index, point in enumerate(field.rows[0].points):
        assert system.field_navigation.current_row.points[index].distance(point) == pytest.approx(0, abs=1e-8)
    assert system.field_navigation.automation_watcher.field_watch_active
    assert system.automator.is_running
    await forward(until=lambda: system.field_navigation._state == FieldNavigationState.FOLLOWING_ROW)
    start_point = field.rows[0].points[0].cartesian()
    assert system.odometer.prediction.point.x == pytest.approx(start_point.x, abs=0.05)
    assert system.odometer.prediction.point.y == pytest.approx(start_point.y, abs=0.05)
    assert system.field_navigation.automation_watcher.field_watch_active


async def test_approaching_first_row_from_other_side(system: System, field: Field):
    # pylint: disable=protected-access
    assert system.gnss.current
    assert system.gnss.current.location.distance(ROBOT_GEO_START_POSITION) < 0.01
    start_point = field.rows[0].points[-1].cartesian()
    end_point = field.rows[0].points[0].cartesian()
    safe_start_point = start_point.polar(-1.0, start_point.direction(end_point))

    async def drive_to_start():
        await system.driver.drive_to(safe_start_point)
    system.automator.start(drive_to_start())
    await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: system.automator.is_stopped)
    assert system.odometer.prediction.point.x == pytest.approx(safe_start_point.x, abs=0.05)
    assert system.odometer.prediction.point.y == pytest.approx(safe_start_point.y, abs=0.05)

    system.field_navigation.field_id = field.id
    system.current_navigation = system.field_navigation
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: system.field_navigation.automation_watcher.field_watch_active)
    await forward(until=lambda: system.current_implement.is_active)
    await forward(until=lambda: system.field_navigation._state == FieldNavigationState.APPROACHING_ROW_START)
    for index, point in enumerate(field.rows[0].points):
        assert system.field_navigation.current_row.points[index].distance(point) == pytest.approx(0, abs=1e-8)
    assert system.field_navigation.automation_watcher.field_watch_active
    assert system.automator.is_running
    await forward(until=lambda: system.field_navigation._state == FieldNavigationState.FOLLOWING_ROW)
    assert system.odometer.prediction.point.x == pytest.approx(start_point.x, abs=0.05)
    assert system.odometer.prediction.point.y == pytest.approx(start_point.y, abs=0.05)
    assert system.field_navigation.automation_watcher.field_watch_active


async def test_approaching_first_row_when_outside_of_field(system: System, field: Field):
    # pylint: disable=protected-access
    assert system.gnss.current
    assert system.gnss.current.location.distance(ROBOT_GEO_START_POSITION) < 0.01
    point_outside = rosys.geometry.Point(x=-10, y=0)

    async def drive_away():
        await system.driver.drive_to(point_outside)
    system.automator.start(drive_away())
    await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: system.automator.is_stopped)
    assert system.odometer.prediction.point.x == pytest.approx(point_outside.x, abs=0.05)
    assert system.odometer.prediction.point.y == pytest.approx(point_outside.y, abs=0.05)

    system.field_navigation.field_id = field.id
    system.current_navigation = system.field_navigation
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: system.field_navigation.automation_watcher.field_watch_active)
    await forward(until=lambda: system.current_implement.is_active)
    for index, point in enumerate(field.rows[0].points):
        assert system.field_navigation.current_row.points[index].distance(point) == pytest.approx(0, abs=1e-8)
    await forward(until=lambda: system.automator.is_stopped)
    assert system.field_navigation._state == FieldNavigationState.APPROACHING_ROW_START
    assert system.odometer.prediction.point.x == pytest.approx(-10, abs=1.0)
    assert system.odometer.prediction.point.y == pytest.approx(0.0, abs=0.5)
    assert not system.automator.is_running, 'should have been stopped because robot is outside of field boundaries'


async def test_complete_row(system: System, field: Field):
    # pylint: disable=protected-access
    assert system.gnss.current
    assert system.gnss.current.location.distance(ROBOT_GEO_START_POSITION) < 0.01
    system.field_navigation.field_id = field.id
    system.current_navigation = system.field_navigation
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: system.field_navigation.automation_watcher.field_watch_active)
    await forward(until=lambda: system.field_navigation._state == FieldNavigationState.APPROACHING_ROW_START)
    for index, point in enumerate(field.rows[0].points):
        assert system.field_navigation.current_row.points[index].distance(point) == pytest.approx(0, abs=1e-8)
    assert system.field_navigation.automation_watcher.field_watch_active
    await forward(until=lambda: system.field_navigation._state == FieldNavigationState.FOLLOWING_ROW)
    await forward(until=lambda: system.field_navigation._state == FieldNavigationState.APPROACHING_ROW_START)
    end_point = field.rows[0].points[1].cartesian()
    assert system.odometer.prediction.point.x == pytest.approx(end_point.x, abs=0.05)
    assert system.odometer.prediction.point.y == pytest.approx(end_point.y, abs=0.05)
    assert system.field_navigation.automation_watcher.field_watch_active


@pytest.mark.skip('TODO: rework in a later PR')
async def test_resuming_field_navigation_after_automation_stop(system: System, field: Field):
    # pylint: disable=protected-access
    system.field_navigation.field = field
    system.current_navigation = system.field_navigation
    system.automator.start()
    await forward(1)  # update gnss reference to use the fields reference
    point = rosys.geometry.Point(x=1.54, y=-6.1)
    await forward(x=point.x, y=point.y, tolerance=0.01)  # drive until we are on first row
    await forward(2)
    assert system.field_navigation._state == FieldNavigationState.FOLLOWING_ROW
    system.automator.stop(because='test')
    await forward(2)
    system.automator.start()
    await forward(5)
    assert system.field_navigation._state == FieldNavigationState.FOLLOWING_ROW
    assert not system.plant_locator.is_paused
    await forward(20)
    assert system.odometer.prediction.point.distance(point) > 0.1


async def test_complete_field(system: System, field: Field):
    # pylint: disable=protected-access
    assert system.gnss.current
    assert system.gnss.current.location.distance(ROBOT_GEO_START_POSITION) < 0.01
    system.field_navigation.field_id = field.id
    system.current_navigation = system.field_navigation
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: system.field_navigation._state == FieldNavigationState.FIELD_COMPLETED, timeout=1500)
    assert system.automator.is_stopped
    end_point = field.rows[-1].points[0].cartesian()
    assert system.odometer.prediction.point.x == pytest.approx(end_point.x, abs=0.05)
    assert system.odometer.prediction.point.y == pytest.approx(end_point.y, abs=0.05)


async def test_complete_field_with_selected_beds(system: System, field_with_beds: Field):
    # pylint: disable=protected-access
    assert system.gnss.current
    assert system.gnss.current.location.distance(ROBOT_GEO_START_POSITION) < 0.01
    system.field_provider.select_field(field_with_beds.id)
    system.field_provider._only_specific_beds = True
    system.field_provider.selected_beds = [1, 3]
    system.current_navigation = system.field_navigation
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: system.field_navigation._state == FieldNavigationState.FIELD_COMPLETED, timeout=1500)
    assert system.automator.is_stopped
    end_point = field_with_beds.rows[2].points[0].cartesian()
    assert system.odometer.prediction.point.x == pytest.approx(end_point.x, abs=0.05)
    assert system.odometer.prediction.point.y == pytest.approx(end_point.y, abs=0.05)


async def test_complete_field_without_first_beds(system: System, field_with_beds: Field):
    # pylint: disable=protected-access
    assert system.gnss.current
    assert system.gnss.current.location.distance(ROBOT_GEO_START_POSITION) < 0.01
    system.field_provider.select_field(field_with_beds.id)
    system.field_provider._only_specific_beds = True
    system.field_provider.selected_beds = [2, 3, 4]
    system.current_navigation = system.field_navigation
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: system.field_navigation._state == FieldNavigationState.FIELD_COMPLETED, timeout=1500)
    assert system.automator.is_stopped
    end_point = field_with_beds.rows[-1].points[1].cartesian()
    assert system.odometer.prediction.point.x == pytest.approx(end_point.x, abs=0.05)
    assert system.odometer.prediction.point.y == pytest.approx(end_point.y, abs=0.05)
