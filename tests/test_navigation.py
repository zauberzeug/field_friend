import json

import numpy as np
import pytest
import rosys
from conftest import ROBOT_GEO_START_POSITION
from rosys.testing import forward

from field_friend import System
from field_friend.automations import Field
from field_friend.automations.implements import Recorder
from field_friend.automations.navigation import StraightLineNavigation
from field_friend.navigation import GnssSimulation


async def test_straight_line(system: System):
    assert system.odometer.prediction.point.x == 0
    assert isinstance(system.current_navigation, StraightLineNavigation)
    assert isinstance(system.current_navigation.implement, Recorder)
    system.automator.start()
    await forward(2)
    assert system.automator.is_running
    await forward(55)
    assert not system.automator.is_running, 'automation should stop after default length'
    assert system.odometer.prediction.point.x == pytest.approx(system.straight_line_navigation.length, abs=0.1)


async def test_follow_crops(system: System, detector: rosys.vision.DetectorSimulation):
    for i in range(10):
        x = i/10.0
        p = rosys.geometry.Point3d(x=x, y=np.sin(x/2), z=0)
        detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='maize', position=p))
    system.current_navigation = system.follow_crops_navigation
    assert isinstance(system.current_navigation.implement, Recorder)
    system.automator.start()
    await forward(2)
    assert system.automator.is_running
    await forward(50)
    assert not system.automator.is_running, 'automation should stop if no crops are detected anymore'
    assert system.odometer.prediction.point.x == pytest.approx(1.4, abs=0.1)
    assert system.odometer.prediction.point.y == pytest.approx(0.6, abs=0.1)
    assert system.odometer.prediction.yaw_deg == pytest.approx(25.0, abs=1.0)


# BETWEEN_ROWS = GeoPoint(lat=51.98316518491446, long=7.434244252776547)
# ON_FIRST_ROW = GeoPoint(lat=51.983168021639834, long=7.434232023041047)


async def test_start_approaching_first_row(system: System, field: Field, gnss: GnssSimulation):
    system.field_navigation.field = field
    print(json.dumps(rosys.persistence.to_dict(field)))
    system.current_navigation = system.field_navigation
    assert system.gnss.current
    assert system.gnss.current.location.distance(ROBOT_GEO_START_POSITION) < 0.01
    system.automator.start()
    await forward(x=1.5, y=-6.05)
    assert system.field_navigation.current_row == field.rows[0]
    assert system.field_navigation.state == system.field_navigation.State.APPROACHING_ROW_START
    await forward(5)
    assert system.automator.is_running
    assert system.field_navigation.current_row == field.rows[0]
    assert system.field_navigation.state == system.field_navigation.State.FOLLOWING_ROW


# async def test_weeding_after_modifying_rows(system: System, field: Field, gnss: GnssSimulation):
#     system.automator.start(system.tools['weeding']())
#     await forward(1)
#     system.automator.stop('change row')
#     await forward(1)
#     system.field_provider.remove_row(field, field.rows[0])
#     system.field_provider.create_row(field, points=[GeoPoint(lat=51.98318416921418, long=7.4342004020500285),
#                                                     GeoPoint(lat=51.98312378543273, long=7.434291470886676)])
#     system.automator.start(system.tools['weeding']())
#     await forward(1)
#     assert system.automator.is_running
