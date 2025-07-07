# TODO: remove when done
import math

import numpy as np
import pytest
import rosys
from conftest import ROBOT_GEO_START_POSITION
from rosys.testing import assert_point, forward

from field_friend import System
from field_friend.automations import Field
from field_friend.automations.implements import WeedingImplement


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
    system.current_navigation.linear_speed_limit = 0.3
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
    await forward(1)
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
    system.current_navigation.linear_speed_limit = 0.3
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: system.field_navigation._state == FieldNavigationState.FIELD_COMPLETED, timeout=300)
    await forward(until=lambda: system.automator.is_stopped)
    end_point = field.rows[-1].points[0].to_local()
    assert_point(system.robot_locator.pose.point, end_point, tolerance=0.05)


async def test_complete_field_with_selected_beds(system: System, field_with_beds: Field):
    # pylint: disable=protected-access
    assert system.gnss.last_measurement
    assert ROBOT_GEO_START_POSITION is not None
    assert system.gnss.last_measurement.point.distance(ROBOT_GEO_START_POSITION) < 0.01
    system.field_provider.select_field(field_with_beds.id)
    system.field_provider.only_specific_beds = True
    system.field_provider.selected_beds = [0, 2]
    system.current_navigation = system.field_navigation
    system.current_navigation.linear_speed_limit = 0.3
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: system.field_navigation._state == FieldNavigationState.FIELD_COMPLETED, timeout=300)
    await forward(until=lambda: system.automator.is_stopped)
    end_point = field_with_beds.rows[2].points[0].to_local()
    assert_point(system.robot_locator.pose.point, end_point, tolerance=0.05)


async def test_complete_field_without_second_bed(system: System, field_with_beds: Field):
    # pylint: disable=protected-access
    assert system.gnss.last_measurement
    assert ROBOT_GEO_START_POSITION is not None
    assert system.gnss.last_measurement.point.distance(ROBOT_GEO_START_POSITION) < 0.01
    system.field_provider.select_field(field_with_beds.id)
    system.field_provider.only_specific_beds = True
    system.field_provider.selected_beds = [0, 2, 3]
    system.current_navigation = system.field_navigation
    system.current_navigation.linear_speed_limit = 0.3
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: system.field_navigation._state == FieldNavigationState.FIELD_COMPLETED, timeout=300)
    await forward(until=lambda: system.automator.is_stopped)
    end_point = field_with_beds.rows[-1].points[1].to_local()
    assert_point(system.robot_locator.pose.point, end_point, tolerance=0.05)


async def test_field_with_first_row_excluded(system: System, field_with_beds: Field):
    assert system.gnss.last_measurement
    assert ROBOT_GEO_START_POSITION is not None
    assert system.gnss.last_measurement.point.distance(ROBOT_GEO_START_POSITION) < 0.01

    system.field_provider.select_field(field_with_beds.id)
    system.field_provider.only_specific_beds = True
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
        await forward(until=lambda: system.field_navigation._state == FieldNavigationState.FOLLOW_ROW, timeout=500)
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
