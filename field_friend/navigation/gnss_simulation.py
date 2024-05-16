import rosys

from .geo_point import GeoPoint
from .gnss import Gnss, GNSSRecord


class GnssSimulation(Gnss):

    def __init__(self, odometer: rosys.driving.Odometer) -> None:
        super().__init__(odometer, 0.0)
        self.allow_connection = True
        self.gps_quality = 8

    async def update(self) -> None:
        if self.device is None:
            return
        if self.reference is None:
            self.reference = GeoPoint(lat=51.983159, long=7.434212)
        pose = self.odometer.prediction
        current_position = self.reference.shifted(pose.point)
        record = GNSSRecord()
        record.timestamp = pose.time
        record.latitude, record.longitude = current_position.tuple
        record.mode = "simulation"  # TODO check for possible values and replace "simulation"
        record.gps_qual = self.gps_quality
        self.ROBOT_POSITION_LOCATED.emit()
        self.ROBOT_POSE_LOCATED.emit(pose)
        self._update_record(record)

    async def try_connection(self) -> None:
        if self.allow_connection:
            self.device = 'simulation'

    def set_reference(self, point: GeoPoint) -> None:
        super().set_reference(point)
        self.record.latitude = point.lat
        self.record.longitude = point.long
        self.ROBOT_POSITION_LOCATED.emit()

    def disconnect(self):
        """Simulate serial disconnection.

        The hardware implementation sets the device to None if it encounters a serial exception.
        Reconnect is not longer possible.
        """
        self.device = None
        self.allow_connection = False
