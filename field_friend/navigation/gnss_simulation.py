from copy import deepcopy

import rosys

from field_friend.navigation.gnss import Gnss, PoseProvider
from field_friend.navigation.point_transformation import cartesian_to_wgs84


class GnssSimulation(Gnss):

    def __init__(self, pose_provider: PoseProvider) -> None:
        super().__init__()
        self.pose_provider = pose_provider

    async def update(self) -> None:
        if self.device is None:
            return
        if self.reference_lat is None:
            self.reference_lat = 51.983159
        if self.reference_lon is None:
            self.reference_lon = 7.434212
        pose = deepcopy(self.pose_provider.pose)
        try:
            pose.time = rosys.time()
        except Exception:
            self.log.error('Pose provider has no time attribute')
            return
        current_position = cartesian_to_wgs84([self.reference_lat, self.reference_lon], [pose.x, pose.y])

        self.record.timestamp = pose.time
        self.record.latitude = current_position[0]
        self.record.longitude = current_position[1]
        self.record.mode = "simulation"  # TODO check for possible values and replace "simulation"
        self.record.gps_qual = 8
        self.ROBOT_POSITION_LOCATED.emit()
        self.ROBOT_POSE_LOCATED.emit(pose)

    async def try_connection(self) -> None:
        self.device = 'simulation'

    def set_reference(self, lat: float, lon: float) -> None:
        self.reference_lat = lat
        self.reference_lon = lon
        self.record.latitude = lat
        self.record.longitude = lon
        self.ROBOT_POSITION_LOCATED.emit()
