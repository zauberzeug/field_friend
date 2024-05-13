import rosys

from field_friend.navigation.gnss import Gnss
from field_friend.navigation.point_transformation import cartesian_to_wgs84


class GnssSimulation(Gnss):

    def __init__(self, odometer: rosys.driving.Odometer) -> None:
        super().__init__(odometer)
        self.allow_connection = True

    async def update(self) -> None:
        if self.device is None:
            return
        if self.reference_lat is None:
            self.reference_lat = 51.983159
        if self.reference_lon is None:
            self.reference_lon = 7.434212
        pose = self.odometer.prediction
        current_position = cartesian_to_wgs84([self.reference_lat, self.reference_lon], pose.point)
        self.record.timestamp = pose.time
        self.record.latitude, self.record.longitude = current_position
        self.record.mode = "simulation"  # TODO check for possible values and replace "simulation"
        self.record.gps_qual = 8
        self.ROBOT_POSITION_LOCATED.emit()
        self.ROBOT_POSE_LOCATED.emit(pose)

    async def try_connection(self) -> None:
        if self.allow_connection:
            self.device = 'simulation'

    def set_reference(self, lat: float, lon: float) -> None:
        self.reference_lat = lat
        self.reference_lon = lon
        self.record.latitude = lat
        self.record.longitude = lon
        self.ROBOT_POSITION_LOCATED.emit()

    def disconnect(self):
        """Simulate serial disconnection.

        The hardware implementation sets the device to None if it encounters a serial exception.
        Reconnect is not longer possible.
        """
        self.device = None
        self.allow_connection = False
