
import logging
from typing import Any, AsyncGenerator, Generator

import pytest
import rosys
from rosys.testing import forward, helpers

from field_friend.automations.field_provider import Field
from field_friend.navigation import GeoPoint, GnssSimulation
from field_friend.system import System

ROBOT_GEO_START_POSITION = GeoPoint(lat=51.983159, long=7.434212)


log = logging.getLogger('field_friend.testing')


@pytest.fixture
async def system(integration) -> AsyncGenerator[System, None]:
    s = System()
    helpers.odometer = s.odometer
    helpers.driver = s.driver
    helpers.automator = s.automator
    yield s


@pytest.fixture
async def gnss(system: System) -> AsyncGenerator[GnssSimulation, None]:
    system.gnss.set_reference(ROBOT_GEO_START_POSITION)
    await forward(1)
    assert system.gnss.device is None, 'device should not be created yet'
    await forward(3)
    assert system.gnss.device is not None, 'device should be created'
    yield system.gnss


@pytest.fixture
async def field(system: System) -> AsyncGenerator[Field, None]:
    points = [[51.98317071260942, 7.43411239981148],
              [51.98307173817015, 7.43425187239752],
              [51.983141020300614, 7.434388662818431],
              [51.98322844759802, 7.43424919023239]]
    geo_points = [GeoPoint.from_list(point) for point in points]
    f = Field(id='test-field',  name='test-field', points=geo_points, reference=geo_points[0])
    system.field_provider.add_field(f)
    yield f


@ pytest.fixture
def mowing(system: System, gnss: GnssSimulation, field: Field) -> Generator[System, None, None]:
    """Start mowing autiomation"""
    system.field_provider.active_field = field
    system.automator.start(system.automations['mowing']())
    yield system


@ pytest.fixture
def driving(system: System) -> Generator[System, None, None]:
    """Drive 10 meters in a straight line"""
    async def automation():
        while system.driver.prediction.point.x < 10.0:
            await system.driver.wheels.drive(0.2, 0)
            await rosys.sleep(0.1)
    system.automator.start(automation())
    yield system


@ pytest.fixture
def gnss_driving(system: System, gnss: GnssSimulation) -> Generator[System, None, None]:
    """Use GNSS to drive 10 meters in a straight line"""
    async def automation():
        while system.driver.prediction.point.x < 10.0:
            await system.driver.wheels.drive(0.2, 0)
            await rosys.sleep(0.1)
    gnss.set_reference(ROBOT_GEO_START_POSITION)
    system.automation_watcher.gnss_watch_active = True
    system.automator.start(automation())
    yield system
