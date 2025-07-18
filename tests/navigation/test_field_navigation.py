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
    system.automator.stop('test done')


@pytest.mark.parametrize('direction', (0, np.pi))
async def test_start_direction(system: System, field: Field, direction: float):
    first_row_start = field.first_row_start.to_local()
    first_row_end = field.first_row_end.to_local()
    distance = first_row_start.distance(first_row_end)
    start_position = first_row_start.polar(distance / 2, first_row_start.direction(first_row_end))
    set_start_pose(system, Pose(x=start_position.x, y=start_position.y, yaw=direction))

    assert system.field_navigation is not None
    system.current_navigation = system.field_navigation
    assert isinstance(system.current_navigation, FieldNavigation)
    assert isinstance(system.current_navigation.implement, Recorder)
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    assert system.current_navigation.target is not None
    if direction == 0:
        assert system.current_navigation.target.x == pytest.approx(first_row_end.x, abs=0.1)
        assert system.current_navigation.target.y == pytest.approx(first_row_end.y, abs=0.1)
    elif direction == np.pi:
        assert system.current_navigation.target.x == pytest.approx(first_row_start.x, abs=0.1)
        assert system.current_navigation.target.y == pytest.approx(first_row_start.y, abs=0.1)
    else:
        raise ValueError('Invalid direction')


async def test_row_change(system: System, field: Field):
    assert system.field_navigation is not None
    system.current_navigation = system.field_navigation
    assert isinstance(system.current_navigation, FieldNavigation)
    assert isinstance(system.current_navigation.implement, Recorder)
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    assert system.current_navigation.target is not None

    row_segments = [segment for segment in system.current_navigation.path if isinstance(segment, RowSegment)]
    turn_segments = [segment for segment in system.current_navigation.path[1:]
                     if not isinstance(segment, RowSegment)]

    assert len(turn_segments) == 3
    assert turn_segments[1].spline.estimated_length() == pytest.approx(2.4611, abs=0.0001)
    assert turn_segments[2].spline.estimated_length() == pytest.approx(2.5500, abs=0.0001)
    assert turn_segments[3].spline.estimated_length() == pytest.approx(2.4611, abs=0.0001)
    assert_point(turn_segments[1].spline.start, row_segments[0].end.point)
    assert_point(turn_segments[3].spline.end, row_segments[1].start.point)


@pytest.mark.skip(reason='Not implemented yet')
async def test_complete_field(system: System, field: Field):
    assert system.field_navigation is not None
    system.current_navigation = system.field_navigation
    assert isinstance(system.current_navigation, FieldNavigation)
    assert isinstance(system.current_navigation.implement, Recorder)
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    assert system.current_navigation.target is not None
    # system.automator.pause('')

    row_segments = [segment for segment in system.current_navigation.path if isinstance(segment, RowSegment)]
    assert len(row_segments) == 4
    assert isinstance(system.current_navigation.path[0], PathSegment)
    turn_segments = [segment for segment in system.current_navigation.path[1:]
                     if not isinstance(segment, RowSegment)]
    assert len(turn_segments) == 3 * 3

    # system.automator.resume()
    # await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: system.automator.is_stopped, timeout=10000.0)
    last_row_start = field.rows[-1].points[0].to_local()
    last_row_end = field.rows[-1].points[-1].to_local()
    assert system.robot_locator.pose.x == pytest.approx(last_row_end.x, abs=0.1)
    assert system.robot_locator.pose.y == pytest.approx(last_row_end.y, abs=0.1)
    assert system.robot_locator.pose.yaw == pytest.approx(last_row_start.direction(last_row_end), abs=0.1)


@pytest.mark.skip(reason='Not implemented yet')
async def test_resume_field(system: System, field: Field):
    pass


# @pytest.mark.skip(reason='Not implemented yet')
async def test_resume_field_after_pause(system: System, field: Field):
    assert system.field_navigation is not None
    system.current_navigation = system.field_navigation
    assert isinstance(system.current_navigation, FieldNavigation)
    assert isinstance(system.current_navigation.implement, Recorder)
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    system.automator.pause('')


@pytest.mark.skip(reason='Not implemented yet')
async def test_resume_field_after_manual_move(system: System, field: Field):
    assert system.field_navigation is not None
    system.current_navigation = system.field_navigation
    assert isinstance(system.current_navigation, FieldNavigation)
    assert isinstance(system.current_navigation.implement, Recorder)
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    system.automator.pause('')
    current_pose = system.robot_locator.pose
    set_start_pose(system, Pose(x=current_pose.x + 1.0, y=current_pose.y, yaw=current_pose.yaw))
    system.automator.resume()
    # TODO: handle error -> not implemented yet


# @pytest.mark.skip(reason='Not implemented yet')
@pytest.mark.parametrize('offset', (0, -0.06))
async def test_between_rows(system: System, field: Field, offset: float):
    assert system.field_navigation is not None
    system.current_navigation = system.field_navigation
    assert isinstance(system.current_navigation, FieldNavigation)
    assert isinstance(system.current_navigation.implement, Recorder)
    set_start_pose(system, Pose(x=1.0, y=offset, yaw=0.0))
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    assert system.current_navigation.target is not None
    # TODO: handle error


@pytest.mark.skip(reason='Not implemented yet')
@pytest.mark.parametrize('heading_degrees', (0, 40))
async def test_heading_deviation(system: System, field: Field, heading_degrees: float):
    pass
