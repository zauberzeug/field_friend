
import logging
from collections.abc import AsyncGenerator, Generator

import pytest
import rosys
from rosys.geometry import Point
from rosys.testing import forward, helpers

from field_friend import localization
from field_friend.automations import Field, Row
from field_friend.interface.components.field_creator import FieldCreator
from field_friend.localization import GeoPoint, GnssSimulation
from field_friend.system import System

ROBOT_GEO_START_POSITION = GeoPoint(lat=51.98333489813455, long=7.434242465994318)

FIELD_FIRST_ROW_START = GeoPoint(lat=51.98333789813455, long=7.434242765994318)
FIELD_FIRST_ROW_END = FIELD_FIRST_ROW_START.shifted(point=Point(x=10, y=0))

log = logging.getLogger('field_friend.testing')


@pytest.fixture
async def system(rosys_integration, request) -> AsyncGenerator[System, None]:
    s = System(getattr(request, 'param', 'rb34'))
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
async def system_with_tornado(rosys_integration, request) -> AsyncGenerator[System, None]:
    s = System(getattr(request, 'param', 'rb34'))
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


class TestField:
    def __init__(self):
        self.id = 'test_field_id'
        self.name = 'Test Field'
        self.first_row_start = FIELD_FIRST_ROW_START
        self.first_row_end = FIELD_FIRST_ROW_END
        self.row_spacing = 0.45
        self.row_count = 4
        self.outline_buffer_width = 2
        self.bed_count = 1
        self.bed_spacing = 0.45
        self.bed_crops = {
            '0': 'sugar_beet',
        }
        self.row_support_points = []
        self.rows = [
            Row(id=f'field_{self.id}_row_1', name='row_1', points=[
                self.first_row_start,
                self.first_row_end
            ], crop=self.bed_crops['0']),
            Row(id=f'field_{self.id}_row_2', name='row_2', points=[
                self.first_row_start.shifted(Point(x=0, y=-0.45)),
                self.first_row_end.shifted(Point(x=0, y=-0.45))
            ], crop=self.bed_crops['0']),
            Row(id=f'field_{self.id}_row_3', name='row_3', points=[
                self.first_row_start.shifted(Point(x=0, y=-0.9)),
                self.first_row_end.shifted(Point(x=0, y=-0.9))
            ], crop=self.bed_crops['0']),
            Row(id=f'field_{self.id}_row_4', name='row_4', points=[
                self.first_row_start.shifted(Point(x=0, y=-1.35)),
                self.first_row_end.shifted(Point(x=0, y=-1.35))
            ], crop=self.bed_crops['0'])
        ]
        self.outline = [
            self.first_row_start.shifted(Point(x=-self.outline_buffer_width, y=self.outline_buffer_width)),
            self.first_row_end.shifted(Point(x=self.outline_buffer_width, y=self.outline_buffer_width)),
            self.first_row_end.shifted(Point(x=self.outline_buffer_width, y=-
                                       self.outline_buffer_width - (self.row_count - 1) * self.row_spacing)),
            self.first_row_start.shifted(Point(x=-self.outline_buffer_width, y=-
                                         self.outline_buffer_width - (self.row_count - 1) * self.row_spacing)),
            self.first_row_start.shifted(Point(x=-self.outline_buffer_width, y=self.outline_buffer_width))
        ]


@pytest.fixture
async def field(system: System) -> AsyncGenerator[TestField, None]:
    test_field = TestField()
    system.field_provider.create_field(Field(
        id=test_field.id,
        name='Test Field',
        first_row_start=test_field.first_row_start,
        first_row_end=test_field.first_row_end,
        row_spacing=test_field.row_spacing,
        row_count=test_field.row_count,
        outline_buffer_width=test_field.outline_buffer_width,
        bed_count=test_field.bed_count,
        bed_spacing=test_field.bed_spacing,
        bed_crops=test_field.bed_crops,
        row_support_points=[]
    ))
    yield test_field


@pytest.fixture
async def field_with_beds(system: System) -> AsyncGenerator[TestField, None]:
    test_field = TestField()
    test_field.row_count = 1
    test_field.bed_count = 4
    test_field.bed_crops = {
        '0': 'sugar_beet',
        '1': 'garlic',
        '2': 'onion',
        '3': 'lettuce'
    }

    system.field_provider.create_field(Field(
        id=test_field.id,
        name='Test Field With Beds',
        first_row_start=test_field.first_row_start,
        first_row_end=test_field.first_row_end,
        row_spacing=test_field.row_spacing,
        row_count=test_field.row_count,
        row_support_points=[],
        outline_buffer_width=test_field.outline_buffer_width,
        bed_count=test_field.bed_count,
        bed_spacing=test_field.bed_spacing,
        bed_crops=test_field.bed_crops
    ))
    yield test_field


@pytest.fixture
async def field_with_beds_tornado(system_with_tornado: System) -> AsyncGenerator[TestField, None]:
    test_field = TestField()
    test_field.row_count = 1
    test_field.bed_count = 4
    test_field.bed_crops = {
        '0': 'sugar_beet',
        '1': 'garlic',
        '2': 'onion',
        '3': 'lettuce'
    }

    system = system_with_tornado
    system.field_provider.create_field(Field(
        id=test_field.id,
        name='Test Field With Beds',
        first_row_start=test_field.first_row_start,
        first_row_end=test_field.first_row_end,
        row_spacing=test_field.row_spacing,
        row_count=test_field.row_count,
        row_support_points=[],
        outline_buffer_width=test_field.outline_buffer_width,
        bed_count=test_field.bed_count,
        bed_spacing=test_field.bed_spacing,
        bed_crops=test_field.bed_crops
    ))
    yield test_field


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
