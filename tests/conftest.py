
import logging
from typing import AsyncGenerator, Generator

import pytest
import rosys
from rosys.testing import forward, helpers

from field_friend.automations import Field
from field_friend.navigation import GeoPoint, GnssSimulation
from field_friend.system import System

ROBOT_GEO_START_POSITION = GeoPoint(lat=51.983173401171236, long=7.434163443756093)

log = logging.getLogger('field_friend.testing')


@pytest.fixture
async def system(integration, request) -> AsyncGenerator[System, None]:
    System.version = getattr(request, 'param', 'rb34')
    s = System()
    s.gnss.reference = ROBOT_GEO_START_POSITION
    helpers.odometer = s.odometer
    helpers.driver = s.driver
    helpers.automator = s.automator
    await forward(1)
    assert s.gnss.device is None, 'device should not be created yet'
    await forward(3)
    assert s.gnss.device is not None, 'device should be created'
    assert s.gnss.current.location.distance(ROBOT_GEO_START_POSITION) == 0
    yield s


@pytest.fixture
def gnss(system: System) -> GnssSimulation:
    assert isinstance(system.gnss, GnssSimulation)
    return system.gnss


@pytest.fixture
async def field(system: System) -> AsyncGenerator[Field, None]:
    points = [[51.98317071260942, 7.43411239981148],
              [51.98307173817015, 7.43425187239752],
              [51.983141020300614, 7.434388662818431],
              [51.98322844759802, 7.43424919023239]]
    geo_points = [GeoPoint.from_list(point) for point in points]
    f = system.field_provider.create_field(points=geo_points)
    system.field_provider.create_row(f, points=[GeoPoint(lat=51.98318416921418, long=7.4342004020500285),
                                                GeoPoint(lat=51.98312378543273, long=7.434291470886676)])
    system.field_provider.active_field = f
    yield f


@pytest.fixture
def driving(system: System) -> Generator[System, None, None]:
    """Drive 10 meters in a straight line"""
    async def automation():
        while system.driver.prediction.point.x < 10.0:
            await system.driver.wheels.drive(0.2, 0)
            await rosys.sleep(0.1)
    system.automator.start(automation())
    yield system


@pytest.fixture
def gnss_driving(system: System) -> Generator[System, None, None]:
    """Use GNSS to drive 10 meters in a straight line"""
    async def automation():
        while system.driver.prediction.point.x < 10.0:
            await system.driver.wheels.drive(0.2, 0)
            await rosys.sleep(0.1)
    system.automation_watcher.gnss_watch_active = True
    system.automator.start(automation())
    yield system


@pytest.fixture
def detector(system: System) -> Generator[rosys.vision.DetectorSimulation, None, None]:
    assert isinstance(system.detector, rosys.vision.DetectorSimulation)
    yield system.detector
