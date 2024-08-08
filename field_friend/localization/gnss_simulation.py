from typing import Optional

import rosys

from .. import localization
from .geo_point import GeoPoint
from .gnss import Gnss, GNSSRecord


class GnssSimulation(Gnss):

    def __init__(self, odometer: rosys.driving.Odometer, wheels: rosys.hardware.WheelsSimulation) -> None:
        super().__init__(odometer, 0.0)
        self.wheels = wheels
        self.allow_connection = True
        self.gps_quality = 4
        self.mode = 'SSSS'

    async def try_connection(self) -> None:
        if self.allow_connection:
            self.device = 'simulation'

    async def _create_new_record(self) -> Optional[GNSSRecord]:
        pose = self.wheels.pose
        if localization.reference.lat == 0 and localization.reference.long == 0:
            new_position = GeoPoint(lat=51.983159, long=7.434212)
        else:
            new_position = localization.reference.shifted(pose.point)
        record = GNSSRecord(timestamp=pose.time, location=new_position)
        record.mode = "simulation"  # TODO check for possible values and replace "simulation"
        record.gps_qual = self.gps_quality
        record.mode = self.mode
        await rosys.sleep(0.1)  # NOTE simulation does not be so fast and only eats a lot of cpu time
        return record

    def disconnect(self):
        """Simulate serial disconnection.

        The hardware implementation sets the device to None if it encounters a serial exception.
        Reconnect is not longer possible.
        """
        self.device = None
        self.allow_connection = False
