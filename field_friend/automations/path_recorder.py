import logging
from typing import Literal, Optional

import rosys
from rosys.driving import PathSegment
from rosys.geometry import Spline

from ..navigation import Gnss
from .path_provider import Path, PathProvider


class PathRecorder:

    def __init__(self, path_provider: PathProvider, driver: rosys.driving.Driver, steerer: rosys.driving.Steerer, gnss: Gnss) -> None:
        self.log = logging.getLogger('field_friend.path_recorder')
        self.path_provider = path_provider
        self.driver = driver
        self.steerer = steerer
        self.gnss = gnss

        self.PATH_DRIVING_STARTED = rosys.event.Event()
        """A path is being driven (argument: list of PathSegments)."""

        self.current_path_recording: str = ''
        self.current_path_driving: str = ''
        self.state: Literal['recording', 'driving', 'idle'] = 'idle'

    async def record_path(self, path: Path) -> None:
        if self.gnss.device is None:
            self.log.warning('not recording because no GNSS device found')
            return
        if self.state != 'idle':
            self.log.warning(f'not recording because state is {self.state}')
            return
        if path is None:
            self.log.warning(f'not recording path "{path.name}" not found')
            return
        self.log.info(f'recording path {path.name}')
        if self.gnss.device != 'simulation':
            ref_lat, ref_lon = await self.gnss.get_reference()
            if ref_lat is None or ref_lon is None:
                self.log.warning('not recording because no reference location set')
                return
            path.reference_lat = ref_lat
            path.reference_lon = ref_lon
        rosys.notify(f'Recording...Please drive the path {path.name} now.')
        self.current_path_recording = path.name
        self.state = 'recording'
        splines = []
        last_pose = self.driver.odometer.prediction
        while self.state == 'recording':
            if self.driver.odometer.current_velocity.linear > 0.05:
                new_pose = self.driver.odometer.prediction
                splines.append(Spline.from_poses(last_pose, new_pose))
                last_pose = new_pose
            else:
                self.log.info('not recording because no movement detected')
            await rosys.sleep(5)
        new_pose = self.driver.odometer.prediction
        splines.append(Spline.from_poses(last_pose, new_pose))
        path.path_segments = [PathSegment(spline=spline) for spline in splines]
        if path.path_segments == []:
            self.log.warning('No path was recorded')
            rosys.notify('No path was recorded', 'negative')
            return
        self.log.info(f'path {path.name} recorded')
        rosys.notify(f'Path {path.name} succesfully recorded', 'positive')
        self.path_provider.invalidate()

    async def drive_path(self, path: Path) -> None:
        self.log.info(f'driving path {path.name}')
        # self.driver.parameters.can_drive_backwards = True
        if self.gnss.device is None:
            self.log.warning('not driving because no GNSS device found')
            return
        if self.state != 'idle':
            self.log.warning(f'not driving because state is {self.state}')
            return
        self.state = 'driving'
        if path is None:
            self.log.warning(f'not driving because path "{path.name}" not found')
            return
        if path == []:
            self.log.warning(f'path {path.name} is empty')
            return
        if self.gnss.device != 'simulation':
            if path.reference_lat is None or path.reference_lon is None:
                self.log.warning('not driving because no reference location set')
                return
            self.gnss.set_reference(path.reference_lat, path.reference_lon)
            distance = self.gnss.calculate_distance(self.gnss.record.latitude, self.gnss.record.longitude)
            if not distance or distance > 10:
                self.log.warning('not driving because distance to reference location is too large')
                rosys.notify('Distance to reference location is too large', 'negative')
                return
        self.log.info(f'path: {path}')
        self.current_path_driving = path.name
        rosys.notify(f'Driving path {path.name}...', 'info')
        self.PATH_DRIVING_STARTED.emit(path.path_segments)
        await self.driver.drive_to(path.path_segments[0].spline.start)
        await self.driver.drive_path(path.path_segments)
        self.state = 'idle'
        self.current_path_driving = ''
        rosys.notify(f'Path {path.name} succesfully driven', 'positive')
        # self.driver.parameters.can_drive_backwards = False
