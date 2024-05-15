import rosys

from .geo_point import GeoPoint
from .gnss import Gnss


class GnssSimulation(Gnss):

    def __init__(self, odometer: rosys.driving.Odometer) -> None:
        super().__init__(odometer)
        self.allow_connection = True

    async def update(self) -> None:
        if self.device is None:
            return
        if self.reference is None:
            self.reference = GeoPoint(lat=51.983159, long=7.434212)
        pose = self.odometer.prediction
        current_position = self.reference.shifted(pose.point)
        self.record.timestamp = pose.time
        self.record.latitude, self.record.longitude = current_position.tuple
        self.record.mode = "simulation"  # TODO check for possible values and replace "simulation"
        self.record.gps_qual = 8
        self.ROBOT_POSITION_LOCATED.emit()
        self.ROBOT_POSE_LOCATED.emit(pose)

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
