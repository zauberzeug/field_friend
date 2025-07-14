import numpy as np
import pytest
from rosys.geometry import Pose
from rosys.testing import forward

from field_friend import System
from field_friend.automations import Field
from field_friend.automations.implements import Recorder
from field_friend.automations.navigation import FieldNavigation

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


@pytest.mark.skip(reason='Not implemented yet')
async def test_row_change(system: System, field: Field):
    pass


@pytest.mark.skip(reason='Not implemented yet')
async def test_complete_field(system: System, field: Field):
    pass


@pytest.mark.skip(reason='Not implemented yet')
async def test_resume_field(system: System, field: Field):
    pass


@pytest.mark.skip(reason='Not implemented yet')
@pytest.mark.parametrize('offset', (0, -0.06))
async def test_between_rows(system: System, field: Field, offset: float):
    pass


@pytest.mark.skip(reason='Not implemented yet')
@pytest.mark.parametrize('heading_degrees', (0, 40))
async def test_heading_deviation(system: System, field: Field, heading_degrees: float):
    pass
