
import logging
from typing import AsyncGenerator, Generator
from uuid import uuid4

import pytest
import rosys
from rosys.testing import forward, helpers

from field_friend import localization
from field_friend.automations import Field
from field_friend.interface.components.field_creator import FieldCreator
from field_friend.localization import GeoPoint, GnssSimulation
from field_friend.system import System

ROBOT_GEO_START_POSITION = GeoPoint(lat=51.98333489813455, long=7.434242465994318)

FIELD_FIRST_ROW_START = GeoPoint(lat=51.98333789813455, long=7.434242765994318)
FIELD_FIRST_ROW_END = GeoPoint(lat=51.98334192260392, long=7.434293309874038)

log = logging.getLogger('field_friend.testing')


@pytest.fixture
async def system(rosys_integration, request) -> AsyncGenerator[System, None]:
    System.version = getattr(request, 'param', 'rb34')
    s = System()
    assert isinstance(s.detector, rosys.vision.DetectorSimulation)
    s.detector.detection_delay = 0.1
    localization.reference = ROBOT_GEO_START_POSITION
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
    f = system.field_provider.create_field(Field(id=str(uuid4()), name='Field 1', first_row_start=FIELD_FIRST_ROW_START,
                                           first_row_end=FIELD_FIRST_ROW_END, row_spacing=0.5, row_number=10))
    yield f


@pytest.fixture
def field_creator(system: System) -> FieldCreator:
    fc = FieldCreator(system)
    fc.first_row_start = FIELD_FIRST_ROW_START
    fc.row_spacing = 0.5
    fc.row_number = 10
    fc.confirm_geometry()
    fc.first_row_end = FIELD_FIRST_ROW_END
    return fc


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
