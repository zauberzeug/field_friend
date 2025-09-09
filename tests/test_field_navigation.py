import numpy as np
import pytest
from conftest import ROBOT_GEO_START_POSITION, set_robot_pose
from rosys.geometry import Pose
from rosys.hardware import BmsSimulation
from rosys.testing import assert_point, forward

from field_friend import System
from field_friend.automations import Field
from field_friend.automations.implements import Recorder, WeedingImplement
from field_friend.automations.navigation import DriveSegment, FieldNavigation, RowSegment


async def test_approach_first_row(system: System, field: Field):
    assert system.field_navigation is not None
    system.current_navigation = system.field_navigation
    assert isinstance(system.current_navigation, FieldNavigation)
    assert isinstance(system.current_navigation.implement, Recorder)
    assert system.gnss is not None
    assert system.gnss.last_measurement is not None
    assert system.gnss.last_measurement.point.distance(ROBOT_GEO_START_POSITION) < 0.01
    set_robot_pose(system, Pose(x=-1.0, y=0.0, yaw=0.0))
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    assert system.current_navigation.current_segment is not None
    first_row_start = field.first_row_start.to_local()
    first_row_end = field.first_row_end.to_local()
    row_direction = first_row_start.direction(first_row_end)

    approach_segment = system.current_navigation.path[0]
    approach_segment_end = first_row_start.polar(-0.1, row_direction)
    assert approach_segment.end.x == pytest.approx(approach_segment_end.x, abs=0.1)
    assert approach_segment.end.y == pytest.approx(approach_segment_end.y, abs=0.1)
    assert approach_segment.end.yaw_deg == pytest.approx(row_direction, abs=0.1)

    alignment_segment = system.current_navigation.path[1]
    assert alignment_segment.start.x == pytest.approx(approach_segment_end.x, abs=0.1)
    assert alignment_segment.start.y == pytest.approx(approach_segment_end.y, abs=0.1)
    assert alignment_segment.end.x == pytest.approx(first_row_start.x, abs=0.1)
    assert alignment_segment.end.y == pytest.approx(first_row_start.y, abs=0.1)
    assert alignment_segment.end.yaw_deg == pytest.approx(row_direction, abs=0.1)

    first_row_segment = system.current_navigation.path[2]
    assert first_row_segment.start.x == pytest.approx(first_row_start.x, abs=0.1)
    assert first_row_segment.start.y == pytest.approx(first_row_start.y, abs=0.1)
    assert first_row_segment.start.yaw_deg == pytest.approx(row_direction, abs=0.1)


async def test_complete_field(system: System, field: Field):
    assert system.field_navigation is not None
    system.current_navigation = system.field_navigation
    assert isinstance(system.current_navigation, FieldNavigation)
    assert isinstance(system.current_navigation.implement, Recorder)
    set_robot_pose(system, Pose(x=-1.0, y=0.0, yaw=0.0))
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    assert system.current_navigation.current_segment is not None
    assert isinstance(system.current_navigation.path[0], DriveSegment)
    approach_length = system.current_navigation.path[0].spline.estimated_length() + \
        system.current_navigation.path[1].spline.estimated_length()
    assert approach_length == pytest.approx(1.0, abs=0.001)
    row_segments = [segment for segment in system.current_navigation.path if isinstance(segment, RowSegment)]
    assert len(row_segments) == 4
    combined_row_length = sum(segment.spline.estimated_length() for segment in row_segments)
    assert combined_row_length == pytest.approx(4 * 10, abs=0.0001)
    turn_segments = [segment for segment in system.current_navigation.path[2:]
                     if not isinstance(segment, RowSegment)]
    assert len(turn_segments) == 4 * 3
    combined_turn_length = sum(segment.spline.estimated_length() for segment in turn_segments)
    assert combined_turn_length == pytest.approx(4 * 2 * 2.4611 + 3 * 2.550 + 1.65, abs=0.001)


async def test_start_second_row(system: System, field: Field):
    assert system.field_navigation is not None
    system.current_navigation = system.field_navigation
    assert isinstance(system.current_navigation, FieldNavigation)
    assert isinstance(system.current_navigation.implement, Recorder)
    system.current_navigation.return_to_start = False
    set_robot_pose(system, Pose(x=1.0, y=-0.5, yaw=0.0))
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    assert system.current_navigation.current_segment is not None
    assert isinstance(system.current_navigation.path[0], RowSegment)

    row_segments = [segment for segment in system.current_navigation.path if isinstance(segment, RowSegment)]
    assert len(row_segments) == 3
    combined_row_length = sum(segment.spline.estimated_length() for segment in row_segments)
    assert combined_row_length == pytest.approx(3 * 10, abs=0.0001)
    turn_segments = [segment for segment in system.current_navigation.path[1:]
                     if not isinstance(segment, RowSegment)]
    assert len(turn_segments) == 2 * 3
    combined_turn_length = sum(segment.spline.estimated_length() for segment in turn_segments)
    assert combined_turn_length == pytest.approx(2 * 2 * 2.461 + 2 * 2.550, abs=0.001)


async def test_outside_of_field(system: System, field: Field):
    assert system.field_navigation is not None
    system.current_navigation = system.field_navigation
    assert isinstance(system.current_navigation, FieldNavigation)
    assert isinstance(system.current_navigation.implement, Recorder)
    set_robot_pose(system, Pose(x=-5, y=0, yaw=0.0))
    system.automator.start()
    await forward(2)
    assert system.automator.is_stopped


async def test_row_change(system: System, field: Field):
    assert system.field_navigation is not None
    system.current_navigation = system.field_navigation
    assert isinstance(system.current_navigation, FieldNavigation)
    assert isinstance(system.current_navigation.implement, Recorder)
    set_robot_pose(system, Pose(x=0.4, y=0.0, yaw=0.0))
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    assert system.current_navigation.current_segment is not None
    row_segments = [segment for segment in system.current_navigation.path if isinstance(segment, RowSegment)]
    turn_segments = [segment for segment in system.current_navigation.path
                     if not isinstance(segment, RowSegment)]
    assert turn_segments[0].spline.estimated_length() == pytest.approx(2.461, abs=0.001)
    assert turn_segments[1].spline.estimated_length() == pytest.approx(2.550, abs=0.001)
    assert turn_segments[2].spline.estimated_length() == pytest.approx(2.461, abs=0.001)
    assert_point(turn_segments[0].start.point, row_segments[0].end.point)
    assert_point(turn_segments[2].end.point, row_segments[1].start.point)


@pytest.mark.parametrize('heading_degrees', (0, 180))
async def test_start_direction(system: System, field: Field, heading_degrees: float):
    heading = np.deg2rad(heading_degrees)
    first_row_start = field.first_row_start.to_local()
    first_row_end = field.first_row_end.to_local()
    distance = first_row_start.distance(first_row_end)
    start_position = first_row_start.polar(distance / 2, first_row_start.direction(first_row_end))
    set_robot_pose(system, Pose(x=start_position.x, y=start_position.y, yaw=heading))
    assert system.field_navigation is not None
    system.current_navigation = system.field_navigation
    assert isinstance(system.current_navigation, FieldNavigation)
    assert isinstance(system.current_navigation.implement, Recorder)
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    assert system.current_navigation.current_segment is not None
    if heading == 0:
        assert system.current_navigation.current_segment.end.x == pytest.approx(first_row_end.x, abs=0.1)
        assert system.current_navigation.current_segment.end.y == pytest.approx(first_row_end.y, abs=0.1)
    elif heading == np.pi:
        assert system.current_navigation.current_segment.end.x == pytest.approx(first_row_start.x, abs=0.1)
        assert system.current_navigation.current_segment.end.y == pytest.approx(first_row_start.y, abs=0.1)
    else:
        raise ValueError('Invalid direction')


@pytest.mark.parametrize('offset', (0, 0.10, -0.10, 0.101))
async def test_between_rows(system: System, field: Field, offset: float):
    # pylint: disable=protected-access
    system.gnss._lat_std_dev = 0.0
    system.gnss._lon_std_dev = 0.0
    system.gnss._heading_std_dev = 0.0
    first_row_start = field.first_row_start.to_local()
    first_row_end = field.first_row_end.to_local()
    direction = first_row_start.direction(first_row_end)
    start_position = first_row_start.polar(0.1, direction).polar(offset, direction + np.pi/2)
    set_robot_pose(system, Pose(x=start_position.x, y=start_position.y, yaw=direction))
    assert system.field_navigation is not None
    system.current_navigation = system.field_navigation
    assert isinstance(system.current_navigation, FieldNavigation)
    assert isinstance(system.current_navigation.implement, Recorder)
    system.automator.start()
    if offset <= FieldNavigation.MAX_DISTANCE_DEVIATION:
        await forward(until=lambda: system.automator.is_running)
    else:
        assert system.automator.is_stopped


@pytest.mark.parametrize('heading_degrees', (0, 15, 16))
async def test_heading_deviation(system: System, field: Field, heading_degrees: float):
    heading = np.deg2rad(heading_degrees)
    first_row_start = field.first_row_start.to_local()
    first_row_end = field.first_row_end.to_local()
    direction = first_row_start.direction(first_row_end)
    start_position = first_row_start.polar(0.1, direction)
    set_robot_pose(system, Pose(x=start_position.x, y=start_position.y, yaw=direction + heading))
    assert system.field_navigation is not None
    system.current_navigation = system.field_navigation
    assert isinstance(system.current_navigation, FieldNavigation)
    assert isinstance(system.current_navigation.implement, Recorder)
    system.automator.start()
    await forward(2)
    if heading <= FieldNavigation.MAX_ANGLE_DEVIATION:
        assert system.automator.is_running
    else:
        assert system.automator.is_stopped


async def test_selected_beds(system: System, field_with_beds: Field):
    system.field_provider.select_field(field_with_beds.id)
    system.field_provider.only_specific_beds = True
    system.field_provider.selected_beds = [0, 2]
    assert system.field_navigation is not None
    system.current_navigation = system.field_navigation
    assert isinstance(system.current_navigation, FieldNavigation)
    assert isinstance(system.current_navigation.implement, Recorder)
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    row_segments = [segment for segment in system.current_navigation.path if isinstance(segment, RowSegment)]
    assert len(row_segments) == 2
    assert row_segments[0].row.id == field_with_beds.rows[0].id
    assert row_segments[1].row.id == field_with_beds.rows[2].id


async def test_bed_crops(system: System, field_with_beds: Field):
    set_robot_pose(system, Pose(x=0.4, y=0.0, yaw=0.0))
    system.field_provider.select_field(field_with_beds.id)
    system.field_provider.only_specific_beds = True
    system.field_provider.selected_beds = [0, 2]
    assert system.field_navigation is not None
    system.current_navigation = system.field_navigation
    assert isinstance(system.current_navigation, FieldNavigation)
    system.current_navigation.return_to_start = False
    system.current_implement = system.implements['Weed Screw']
    assert isinstance(system.current_implement, WeedingImplement)
    started_segments = 0

    def count_started_segments(_: DriveSegment) -> None:
        nonlocal started_segments
        started_segments += 1
    system.current_navigation.SEGMENT_STARTED.register(count_started_segments)
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: started_segments == 1)
    await forward(2)
    assert system.current_implement.cultivated_crop == field_with_beds.bed_crops[str(0)]

    await forward(until=lambda: started_segments == 5, timeout=200)
    await forward(2)
    assert system.current_implement.cultivated_crop == field_with_beds.bed_crops[str(2)]
    await forward(until=lambda: system.automator.is_stopped)


async def test_start_from_charging_station(system: System, field: Field):
    assert system.field_navigation is not None
    system.current_navigation = system.field_navigation
    assert isinstance(system.current_navigation, FieldNavigation)
    assert isinstance(system.current_navigation.implement, Recorder)
    assert system.current_navigation.field is not None
    assert system.current_navigation.field.charge_dock_pose is not None
    set_robot_pose(system, system.current_navigation.field.charge_dock_pose.to_local())
    system.current_navigation.charge_automatically = True
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    assert system.current_navigation.path[0].spline.estimated_length() == pytest.approx(2.1461685074558305, abs=0.001)
    assert system.current_navigation.path[1].spline.estimated_length() == pytest.approx(0.1, abs=0.001)


async def test_charge_after_field(system: System, field: Field):
    assert system.field_navigation is not None
    system.current_navigation = system.field_navigation
    assert isinstance(system.current_navigation, FieldNavigation)
    assert isinstance(system.current_navigation.implement, Recorder)
    assert system.current_navigation.field is not None
    assert system.current_navigation.field.charge_dock_pose is not None
    set_robot_pose(system, system.current_navigation.field.charge_dock_pose.to_local())
    system.current_navigation.charge_automatically = True
    system.current_navigation.start_row_index = 2
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: system.field_friend.bms.state.is_charging, timeout=350)
    await forward(until=lambda: system.automator.is_stopped)


async def test_charge_in_between_rows(system: System, field: Field):
    assert system.field_navigation is not None
    system.current_navigation = system.field_navigation
    assert isinstance(system.current_navigation, FieldNavigation)
    assert isinstance(system.current_navigation.implement, Recorder)
    assert system.current_navigation.field is not None
    assert system.current_navigation.field.charge_dock_pose is not None
    set_robot_pose(system, system.current_navigation.field.charge_dock_pose.to_local())
    system.current_navigation.charge_automatically = True
    assert isinstance(system.field_friend.bms, BmsSimulation)
    system.field_friend.bms.voltage_per_second = -0.1
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: system.field_friend.bms.state.is_charging, timeout=350)
    await forward(until=lambda: not system.field_friend.bms.state.is_charging, timeout=150)
    await forward(until=lambda: system.field_friend.bms.state.is_charging, timeout=400)
    await forward(until=lambda: system.automator.is_stopped)
