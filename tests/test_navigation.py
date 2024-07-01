import numpy as np
import pytest
from conftest import ROBOT_GEO_START_POSITION

import rosys
from field_friend import System
from field_friend.automations import Field
from field_friend.automations.implements import Recorder
from field_friend.automations.navigation import StraightLineNavigation
from rosys.testing import forward


async def test_straight_line(system: System):
    assert system.odometer.prediction.point.x == 0
    assert isinstance(system.current_navigation, StraightLineNavigation)
    assert isinstance(system.current_navigation.implement, Recorder)
    system.automator.start()
    await forward(2)
    assert system.automator.is_running
    await forward(55)
    assert not system.automator.is_running, 'automation should stop after default length'
    assert system.odometer.prediction.point.x == pytest.approx(system.straight_line_navigation.length, abs=0.1)


async def test_follow_crops_empty(system: System, detector: rosys.vision.DetectorSimulation):
    system.current_navigation = system.follow_crops_navigation
    assert isinstance(system.current_navigation.implement, Recorder)
    system.automator.start()
    await forward(2)
    assert system.automator.is_running
    await forward(50)
    assert not system.automator.is_running, 'automation should stop if no crops are detected anymore'
    assert system.odometer.prediction.point.x == pytest.approx(0.5, abs=0.1)
    assert system.odometer.prediction.point.y == pytest.approx(0, abs=0.01)
    assert system.odometer.prediction.yaw_deg == pytest.approx(0, abs=1.0)


async def test_follow_crops_straight(system: System, detector: rosys.vision.DetectorSimulation):
    for i in range(10):
        x = i/10
        p = rosys.geometry.Point3d(x=x, y=0, z=0)
        detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='maize', position=p))
    system.current_navigation = system.follow_crops_navigation
    assert isinstance(system.current_navigation.implement, Recorder)
    system.automator.start()
    await forward(2)
    assert system.automator.is_running
    await forward(50)
    assert not system.automator.is_running, 'automation should stop if no crops are detected anymore'
    assert system.odometer.prediction.point.x == pytest.approx(1.5, abs=0.1)
    assert system.odometer.prediction.point.y == pytest.approx(0, abs=0.1)
    assert system.odometer.prediction.yaw_deg == pytest.approx(0, abs=1.0)


async def test_follow_crops_sinus(system: System, detector: rosys.vision.DetectorSimulation):
    for i in range(10):
        x = i/10.0
        p = rosys.geometry.Point3d(x=x, y=np.sin(x/2), z=0)
        detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='maize', position=p))
    system.current_navigation = system.follow_crops_navigation
    assert isinstance(system.current_navigation.implement, Recorder)
    system.automator.start()
    await forward(2)
    assert system.automator.is_running
    await forward(150)
    assert not system.automator.is_running, 'automation should stop if no crops are detected anymore'
    assert system.odometer.prediction.point.x == pytest.approx(1.4, abs=0.1)
    assert system.odometer.prediction.point.y == pytest.approx(0.6, abs=0.1)
    assert system.odometer.prediction.yaw_deg == pytest.approx(25.0, abs=5.0)


async def test_follow_crops_right(system: System, detector: rosys.vision.DetectorSimulation):
    for i in range(3):
        x = i*0.4
        y = i/10
        p = rosys.geometry.Point3d(x=x, y=y, z=0)
        detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='maize', position=p))
    system.current_navigation = system.follow_crops_navigation
    assert isinstance(system.current_navigation.implement, Recorder)
    system.automator.start()
    await forward(2)
    assert system.automator.is_running
    await forward(50)
    assert not system.automator.is_running, 'automation should stop if no crops are detected anymore'
    assert system.odometer.prediction.point.x == pytest.approx(1.7, abs=0.1)
    assert system.odometer.prediction.point.y == pytest.approx(0, abs=0.01)
    assert system.odometer.prediction.yaw_deg == pytest.approx(0, abs=1.0)


async def test_follow_crops_left(system: System, detector: rosys.vision.DetectorSimulation):
    for i in range(3):
        x = i*0.4
        y = -(i/10)
        p = rosys.geometry.Point3d(x=x, y=y, z=0)
        detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='maize', position=p))
    system.current_navigation = system.follow_crops_navigation
    assert isinstance(system.current_navigation.implement, Recorder)
    system.automator.start()
    await forward(2)
    assert system.automator.is_running
    await forward(50)
    assert not system.automator.is_running, 'automation should stop if no crops are detected anymore'
    assert system.odometer.prediction.point.x == pytest.approx(1.7, abs=0.1)
    assert system.odometer.prediction.point.y == pytest.approx(0.0, abs=0.01)
    assert system.odometer.prediction.yaw_deg == pytest.approx(0, abs=1.0)


async def test_follow_crops_outlier(system: System, detector: rosys.vision.DetectorSimulation):
    for i in range(10):
        x = i/10
        y = 0
        p = rosys.geometry.Point3d(x=x, y=y, z=0)
        detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='maize', position=p))
    outlier = rosys.geometry.Point3d(x=1.1, y=0.2, z=0)
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='maize', position=outlier))
    for i in range(10):
        x = i/10 + 1.1
        y = 0
        p = rosys.geometry.Point3d(x=x, y=y, z=0)
        detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='maize', position=p))
    system.current_navigation = system.follow_crops_navigation
    assert isinstance(system.current_navigation.implement, Recorder)
    system.automator.start()
    await forward(2)
    assert system.automator.is_running
    await forward(100)
    assert not system.automator.is_running, 'automation should stop if no crops are detected anymore'
    assert system.odometer.prediction.point.x == pytest.approx(2.6, abs=0.1)
    assert system.odometer.prediction.point.y == pytest.approx(0, abs=0.01)
    assert system.odometer.prediction.yaw_deg == pytest.approx(0, abs=1.0)


async def test_follow_crops_outlier_last(system: System, detector: rosys.vision.DetectorSimulation):
    for i in range(20):
        x = i/10
        y = 0
        p = rosys.geometry.Point3d(x=x, y=y, z=0)
        detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='maize', position=p))
    outlier = rosys.geometry.Point3d(x=2.1, y=-0.2, z=0)
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='maize', position=outlier))
    system.current_navigation = system.follow_crops_navigation
    assert isinstance(system.current_navigation.implement, Recorder)
    system.automator.start()
    await forward(2)
    assert system.automator.is_running
    await forward(100)
    assert not system.automator.is_running, 'automation should stop if no crops are detected anymore'
    assert system.odometer.prediction.point.x == pytest.approx(2.6, abs=0.1)
    assert system.odometer.prediction.point.y == pytest.approx(0, abs=0.01)
    assert system.odometer.prediction.yaw_deg == pytest.approx(0, abs=1.0)


async def test_approaching_first_row(system: System, field: Field):
    system.field_navigation.field = field
    system.current_navigation = system.field_navigation
    assert system.gnss.current
    assert system.gnss.current.location.distance(ROBOT_GEO_START_POSITION) < 0.01
    system.automator.start()
    await forward(x=1.5, y=-6.05)
    assert system.field_navigation.current_row == field.rows[0]
    assert system.field_navigation.state == system.field_navigation.State.APPROACHING_ROW_START
    assert system.field_navigation.automation_watcher.field_watch_active
    await forward(5)
    assert system.automator.is_running
    assert system.field_navigation.current_row == field.rows[0]
    assert system.field_navigation.state == system.field_navigation.State.FOLLOWING_ROW
    assert system.field_navigation.automation_watcher.field_watch_active


async def test_not_approaching_first_row_when_outside_field(system: System, field: Field):
    async def drive_away():
        await system.driver.drive_to(rosys.geometry.Point(x=-5, y=0))
    system.automator.start(drive_away())
    await forward(22)
    assert not system.automator.is_running

    system.field_navigation.field = field
    system.current_navigation = system.field_navigation
    system.automator.start()
    await forward(2)
    assert system.field_navigation.current_row == field.rows[0]
    assert system.field_navigation.state == system.field_navigation.State.APPROACHING_ROW_START
    assert not system.automator.is_running, 'should have been stopped because robot is outside of field boundaries'


async def test_resuming_field_navigation_after_automation_stop(system: System, field: Field):
    system.field_navigation.field = field
    system.current_navigation = system.field_navigation
    system.automator.start()
    assert field.reference
    await forward(1)  # update gnss reference to use the fields reference
    point = rosys.geometry.Point(x=1.50, y=-6.1)
    await forward(x=point.x, y=point.y, tolerance=0.01)  # drive until we are on first row
    assert system.field_navigation.state == system.field_navigation.State.FOLLOWING_ROW
    system.automator.stop(because='test')
    await forward(2)
    system.automator.start()
    await forward(5)
    assert system.field_navigation.state == system.field_navigation.State.FOLLOWING_ROW
    assert not system.plant_locator.is_paused
    await forward(20)
    assert system.odometer.prediction.point.distance(point) > 0.1


async def test_coverage_navigation(system: System, field: Field):
    system.field_navigation.field = field
    system.current_navigation = system.field_navigation
    system.automator.start()
    # TODO implement test
