import logging
from collections.abc import AsyncGenerator, Generator

import pytest
import rosys
from rosys.geometry import GeoPoint, GeoReference
from rosys.hardware import GnssSimulation
from rosys.persistence import Persistable
from rosys.testing import forward, helpers

from field_friend.automations.computed_field import ComputedField
from field_friend.automations.field_description import FieldDescription
from field_friend.hardware.double_wheels import WheelsSimulationWithAcceleration
from field_friend.interface.components.field_creator import FieldCreator
from field_friend.system import System

GEO_REFERENCE = GeoReference(GeoPoint.from_degrees(lat=51.98333489813455, lon=7.434242465994318))
ROBOT_GEO_START_POSITION = GEO_REFERENCE.origin

FIELD_FIRST_ROW_START = GeoPoint.from_degrees(lat=51.98333789813455, lon=7.434242765994318)
FIELD_FIRST_ROW_END = FIELD_FIRST_ROW_START.shift_by(x=10, y=0)

log = logging.getLogger('field_friend.testing')


@pytest.fixture
async def system(rosys_integration, request) -> AsyncGenerator[System, None]:
    # TODO: solve in RoSys
    Persistable.instances.clear()
    s = System(getattr(request, 'param', 'u6'), restore_persistence=False)
    assert isinstance(s.detector, rosys.vision.DetectorSimulation)
    s.detector.detection_delay = 0.1
    GeoReference.update_current(GEO_REFERENCE)
    helpers.odometer = s.robot_locator
    helpers.driver = s.driver
    helpers.automator = s.automator
    await forward(3)
    assert s.gnss.is_connected, 'device should be created'
    assert s.gnss.last_measurement is not None
    assert GeoReference.current is not None
    assert s.gnss.last_measurement.point.distance(GeoReference.current.origin) == pytest.approx(0, abs=1e-8)
    yield s


@pytest.fixture
async def system_with_tornado(rosys_integration, request) -> AsyncGenerator[System, None]:
    # TODO: solve in RoSys
    Persistable.instances.clear()
    s = System(getattr(request, 'param', 'u4'), restore_persistence=False)
    assert isinstance(s.detector, rosys.vision.DetectorSimulation)
    s.detector.detection_delay = 0.1
    GeoReference.update_current(GEO_REFERENCE)
    helpers.odometer = s.robot_locator
    helpers.driver = s.driver
    helpers.automator = s.automator
    await forward(3)
    assert s.gnss.is_connected, 'device should be created'
    assert s.gnss.last_measurement is not None
    assert GeoReference.current is not None
    assert s.gnss.last_measurement.point.distance(GeoReference.current.origin) == pytest.approx(0, abs=1e-8)
    yield s


@pytest.fixture
async def system_with_acceleration(rosys_integration) -> AsyncGenerator[System, None]:
    # TODO: solve in RoSys
    Persistable.instances.clear()
    s = System('u4', restore_persistence=False, use_acceleration=True)
    assert isinstance(s.field_friend.wheels, WheelsSimulationWithAcceleration)
    assert isinstance(s.detector, rosys.vision.DetectorSimulation)
    s.detector.detection_delay = 0.1
    GeoReference.update_current(GEO_REFERENCE)
    helpers.odometer = s.robot_locator
    helpers.driver = s.driver
    helpers.automator = s.automator
    await forward(3)
    assert s.gnss.is_connected, 'device should be created'
    assert s.gnss.last_measurement is not None
    assert GeoReference.current is not None
    assert s.gnss.last_measurement.point.distance(GeoReference.current.origin) == pytest.approx(0, abs=1e-8)
    yield s


@pytest.fixture
def gnss(system: System) -> GnssSimulation:
    assert isinstance(system.gnss, GnssSimulation)
    return system.gnss


@pytest.fixture
async def field(system: System) -> AsyncGenerator[ComputedField, None]:
    field_description = FieldDescription(
        id='test_field_id',
        name='Test Field',
        first_row_start=FIELD_FIRST_ROW_START,
        first_row_end=FIELD_FIRST_ROW_END,
        row_spacing=0.45,
        row_count=4,
        outline_buffer_width=2,
        bed_count=1,
        bed_spacing=0.45,
        bed_crops={'0': 'sugar_beet'},
        row_support_points=[]
    )
    computed_field = system.field_provider.add_field_description(field_description)
    yield computed_field


@pytest.fixture
async def field_with_beds(system: System) -> AsyncGenerator[ComputedField, None]:
    field_description = FieldDescription(
        id='test_field_id',
        name='Test Field With Beds',
        first_row_start=FIELD_FIRST_ROW_START,
        first_row_end=FIELD_FIRST_ROW_END,
        row_spacing=0.45,
        row_count=1,
        outline_buffer_width=2,
        bed_count=4,
        bed_spacing=0.45,
        bed_crops={'0': 'sugar_beet', '1': 'garlic', '2': 'onion', '3': 'lettuce'}
    )
    computed_field = system.field_provider.add_field_description(field_description)
    yield computed_field


@pytest.fixture
async def field_with_beds_tornado(system_with_tornado: System) -> AsyncGenerator[ComputedField, None]:
    field_description = FieldDescription(
        id='test_field_id',
        name='Test Field With Beds',
        first_row_start=FIELD_FIRST_ROW_START,
        first_row_end=FIELD_FIRST_ROW_END,
        row_spacing=0.45,
        row_count=1,
        outline_buffer_width=2,
        bed_count=4,
        bed_spacing=0.45,
        bed_crops={'0': 'sugar_beet', '1': 'garlic', '2': 'onion', '3': 'lettuce'}
    )
    computed_field = system_with_tornado.field_provider.add_field_description(field_description)
    yield computed_field


@pytest.fixture
def field_creator(system: System) -> FieldCreator:
    fc = FieldCreator(system)
    fc.first_row_start = FIELD_FIRST_ROW_START
    fc.row_spacing = 0.5
    fc.row_count = 10
    fc.bed_count = 2
    fc.bed_spacing = 1.0
    fc.bed_crops = {
        '0': 'sugar_beet',
        '1': 'garlic'
    }
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
