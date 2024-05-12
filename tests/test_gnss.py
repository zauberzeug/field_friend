import rosys
from rosys.testing import assert_point, forward

from field_friend.automations.field_provider import Field
from field_friend.navigation.gnss_simulation import GnssSimulation
from field_friend.system import System


async def test_driving_with_gnss(system: System, gnss: GnssSimulation, field: Field):
    system.field_provider.active_field = field
    system.automator.start(system.automations['mowing']())
    await forward(6)
    assert_point(system.odometer.prediction.point, rosys.geometry.Point(x=-0.14, y=-1.17))
