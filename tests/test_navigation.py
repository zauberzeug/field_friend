import numpy as np
import pytest
import rosys
from rosys.testing import forward

from field_friend import System
from field_friend.automations.implements import Recorder
from field_friend.automations.navigation import StraightLineNavigation


async def test_straight_line(system: System):
    assert system.odometer.prediction.point.x == 0
    assert isinstance(system.current_navigation, StraightLineNavigation)
    assert isinstance(system.current_navigation.implement, Recorder)
    system.automator.start(system.current_navigation.start())
    await forward(2)
    assert system.automator.is_running
    await forward(25)
    assert not system.automator.is_running, 'automation should stop after default length'
    assert system.odometer.prediction.point.x == pytest.approx(system.straight_line_navigation.length, abs=0.1)


async def test_follow_crops(system: System, detector: rosys.vision.DetectorSimulation):
    for i in range(10):
        x = i/10.0
        p = rosys.geometry.Point3d(x=x, y=np.sin(x/2), z=0)
        detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='maize', position=p))
    system.current_navigation = system.follow_crops_navigation
    assert isinstance(system.current_navigation.implement, Recorder)
    system.automator.start(system.follow_crops_navigation.start())
    await forward(2)
    assert system.automator.is_running
    await forward(30)
    assert not system.automator.is_running, 'automation should stop if no crops are detected anymore'
    assert system.odometer.prediction.point.x == pytest.approx(1.4, abs=0.1)
    assert system.odometer.prediction.point.y == pytest.approx(0.5, abs=0.1)
    assert system.odometer.prediction.yaw_deg == pytest.approx(25.0, abs=1.0)


# async def test_start_weeding_auto_selects_rows(system: System, field: Field, gnss: GnssSimulation):
#     system.automator.start(system.straight_line_navigation.start())
#     await forward(1)
#     assert system.automator.is_running
#     assert system.weeding.start_row_id == field.rows[0].id
#     assert system.weeding.end_row_id == field.rows[-1].id


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
