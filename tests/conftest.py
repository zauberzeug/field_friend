
from typing import Any, AsyncGenerator

import pytest
from rosys.testing import forward, helpers

from field_friend.automations.field_provider import Field
from field_friend.navigation.gnss import GnssSimulation
from field_friend.system import System

ROBOT_GEO_START_POSITION = [51.983159, 7.434212]


@pytest.fixture
async def system(integration) -> AsyncGenerator[Any, System]:
    s = System()
    helpers.odometer = s.odometer
    helpers.driver = s.driver
    helpers.automator = s.automator
    yield s


@pytest.fixture
async def gnss(system: System) -> AsyncGenerator[Any, GnssSimulation]:
    system.gnss.set_reference(ROBOT_GEO_START_POSITION[0], ROBOT_GEO_START_POSITION[1])
    await forward(1)
    assert system.gnss.device is None, 'device should not be created yet'
    await forward(3)
    assert system.gnss.device is not None, 'device should be created'
    yield system.gnss


@pytest.fixture
async def field(system: System) -> AsyncGenerator[Any, Field]:
    point_list = [[51.98317071260942, 7.43411239981148],
                  [51.98307173817015, 7.43425187239752],
                  [51.983141020300614, 7.434388662818431],
                  [51.98322844759802, 7.43424919023239]]
    f = Field(id='test-field', name='test-field',
              outline_wgs84=point_list, reference_lat=point_list[0][0], reference_lon=point_list[0][1])
    system.field_provider.add_field(f)
    yield f
