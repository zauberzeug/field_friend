import numpy as np
import pytest
from rosys.geometry import Pose
from rosys.testing import assert_point, forward

from field_friend import System
from field_friend.automations import Field
from field_friend.automations.implements import Recorder
from field_friend.automations.navigation import FieldNavigation, PathSegment, RowSegment

from ..conftest import ROBOT_GEO_START_POSITION, set_start_pose


async def test_approach_first_row(system: System, field: Field):
    assert system.field_navigation is not None
    system.current_navigation = system.field_navigation
    assert isinstance(system.current_navigation, FieldNavigation)
    assert isinstance(system.current_navigation.implement, Recorder)
    assert system.gnss is not None
    assert system.gnss.last_measurement is not None
    assert system.gnss.last_measurement.point.distance(ROBOT_GEO_START_POSITION) < 0.01
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    assert system.current_navigation.target is not None
    first_row_start = field.first_row_start.to_local()
    first_row_end = field.first_row_end.to_local()
    assert system.current_navigation.target.point.x == pytest.approx(first_row_start.x, abs=0.1)
    assert system.current_navigation.target.point.y == pytest.approx(first_row_start.y, abs=0.1)
    assert system.current_navigation.target.yaw_deg == pytest.approx(first_row_start.direction(first_row_end), abs=0.1)


async def test_complete_field(system: System, field: Field):
    assert system.field_navigation is not None
    system.current_navigation = system.field_navigation
    assert isinstance(system.current_navigation, FieldNavigation)
    assert isinstance(system.current_navigation.implement, Recorder)
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    assert system.current_navigation.target is not None
    assert isinstance(system.current_navigation.path[0], PathSegment)
    assert system.current_navigation.path[0].spline.estimated_length() == pytest.approx(0.3345, abs=0.0001)
    row_segments = [segment for segment in system.current_navigation.path if isinstance(segment, RowSegment)]
    assert len(row_segments) == 4
    combined_row_length = sum(segment.spline.estimated_length() for segment in row_segments)
    assert combined_row_length == pytest.approx(4 * 10, abs=0.0001)
    turn_segments = [segment for segment in system.current_navigation.path[1:]
                     if not isinstance(segment, RowSegment)]
    assert len(turn_segments) == 3 * 3
    combined_turn_length = sum(segment.spline.estimated_length() for segment in turn_segments)
    assert combined_turn_length == pytest.approx(3 * 2 * 2.461 + 3 * 2.550, abs=0.001)


async def test_start_second_row(system: System, field: Field):
    assert system.field_navigation is not None
    system.current_navigation = system.field_navigation
    assert isinstance(system.current_navigation, FieldNavigation)
    assert isinstance(system.current_navigation.implement, Recorder)
    set_start_pose(system, Pose(x=1.0, y=-0.5, yaw=0.0))
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    assert system.current_navigation.target is not None
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
    set_start_pose(system, Pose(x=-5, y=0, yaw=0.0))
    system.automator.start()
    await forward(1)
    assert not system.automator.is_running


async def test_row_change(system: System, field: Field):
    assert system.field_navigation is not None
    system.current_navigation = system.field_navigation
    assert isinstance(system.current_navigation, FieldNavigation)
    assert isinstance(system.current_navigation.implement, Recorder)
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    assert system.current_navigation.target is not None
    row_segments = [segment for segment in system.current_navigation.path if isinstance(segment, RowSegment)]
    turn_segments = [segment for segment in system.current_navigation.path
                     if not isinstance(segment, RowSegment)]
    assert turn_segments[1].spline.estimated_length() == pytest.approx(2.461, abs=0.001)
    assert turn_segments[2].spline.estimated_length() == pytest.approx(2.550, abs=0.001)
    assert turn_segments[3].spline.estimated_length() == pytest.approx(2.461, abs=0.001)
    assert_point(turn_segments[1].spline.start, row_segments[0].end.point)
    assert_point(turn_segments[3].spline.end, row_segments[1].start.point)


@pytest.mark.parametrize('heading_degrees', (0, 180))
async def test_start_direction(system: System, field: Field, heading_degrees: float):
    heading = np.deg2rad(heading_degrees)
    first_row_start = field.first_row_start.to_local()
    first_row_end = field.first_row_end.to_local()
    distance = first_row_start.distance(first_row_end)
    start_position = first_row_start.polar(distance / 2, first_row_start.direction(first_row_end))
    set_start_pose(system, Pose(x=start_position.x, y=start_position.y, yaw=heading))
    assert system.field_navigation is not None
    system.current_navigation = system.field_navigation
    assert isinstance(system.current_navigation, FieldNavigation)
    assert isinstance(system.current_navigation.implement, Recorder)
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    assert system.current_navigation.target is not None
    if heading == 0:
        assert system.current_navigation.target.x == pytest.approx(first_row_end.x, abs=0.1)
        assert system.current_navigation.target.y == pytest.approx(first_row_end.y, abs=0.1)
    elif heading == np.pi:
        assert system.current_navigation.target.x == pytest.approx(first_row_start.x, abs=0.1)
        assert system.current_navigation.target.y == pytest.approx(first_row_start.y, abs=0.1)
    else:
        raise ValueError('Invalid direction')


@pytest.mark.parametrize('offset', (0, 0.10, 0.11))
async def test_between_rows(system: System, field: Field, offset: float):
    first_row_start = field.first_row_start.to_local()
    first_row_end = field.first_row_end.to_local()
    direction = first_row_start.direction(first_row_end)
    start_position = first_row_start.polar(0.1, direction).polar(offset, direction + np.pi/2)
    set_start_pose(system, Pose(x=start_position.x, y=start_position.y, yaw=direction))
    assert system.field_navigation is not None
    system.current_navigation = system.field_navigation
    assert isinstance(system.current_navigation, FieldNavigation)
    assert isinstance(system.current_navigation.implement, Recorder)
    system.automator.start()
    await forward(1)
    if offset <= FieldNavigation.MAX_DISTANCE_DEVIATION:
        assert system.automator.is_running
    else:
        assert not system.automator.is_running


@pytest.mark.parametrize('heading_degrees', (0, 15, 16))
async def test_heading_deviation(system: System, field: Field, heading_degrees: float):
    heading = np.deg2rad(heading_degrees)
    first_row_start = field.first_row_start.to_local()
    first_row_end = field.first_row_end.to_local()
    direction = first_row_start.direction(first_row_end)
    start_position = first_row_start.polar(0.1, direction)
    set_start_pose(system, Pose(x=start_position.x, y=start_position.y, yaw=direction + heading))
    assert system.field_navigation is not None
    system.current_navigation = system.field_navigation
    assert isinstance(system.current_navigation, FieldNavigation)
    assert isinstance(system.current_navigation.implement, Recorder)
    system.automator.start()
    await forward(1)
    if heading <= FieldNavigation.MAX_ANGLE_DEVIATION:
        assert system.automator.is_running
    else:
        assert not system.automator.is_running


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
