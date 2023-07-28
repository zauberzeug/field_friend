import logging
from dataclasses import dataclass, field
from typing import Any, Literal, Optional

import rosys
from rosys.driving import PathSegment
from rosys.geometry import Spline

from . import Gnss


@dataclass(slots=True, kw_only=True)
class Path:
    name: str
    path: list[rosys.driving.PathSegment] = field(default_factory=list)
    reference_lat: Optional[float] = None
    reference_lon: Optional[float] = None


class PathProvider:
    def __init__(self, driver: rosys.driving.Driver, steerer: rosys.driving.Steerer, gnss: Gnss) -> None:
        self.log = logging.getLogger('field_friend.path_recorder')
        self.driver = driver
        self.steerer = steerer
        self.gnss = gnss
        self.PATHS_CHANGED = rosys.event.Event()
        """The dict of paths has changed."""

        self.PATH_DRIVING_STARTED = rosys.event.Event()
        """A path is being driven (argument: list of PathSegments)."""

        self.paths: list[Path] = []
        self.current_path_recording: str = ''
        self.state: Literal['recording', 'driving', 'idle'] = 'idle'
        self.needs_backup: bool = False
        rosys.persistence.register(self)

    def backup(self) -> dict:
        return {'paths': rosys.persistence.to_dict(self.paths)}

    def restore(self, data: dict[str, Any]) -> None:
        rosys.persistence.replace_list(self.paths, Path, data.get('paths', []))

    def invalidate(self) -> None:
        self.needs_backup = True
        self.PATHS_CHANGED.emit()

    def add_path(self, path: Path) -> None:
        self.paths.append(path)
        self.invalidate()

    def remove_path(self, path: Path) -> None:
        self.paths.remove(path)
        self.invalidate()

    def clear_paths(self) -> None:
        self.paths.clear()
        self.invalidate()

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
            ref_lat, ref_lon = await self.get_reference()
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
        path.path = [PathSegment(spline=spline) for spline in splines]
        if path.path == []:
            self.log.warning('No path was recorded')
            rosys.notify('No path was recorded', 'negative')
            return
        self.log.info(f'path {path.name} recorded')
        rosys.notify(f'Path {path.name} succesfully recorded', 'positive')
        self.invalidate()

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
            if distance and distance > 10:
                self.log.warning('not driving because distance to reference location is too large')
                rosys.notify('Distance to reference location is too large', 'negative')
                return
        self.log.info(f'path: {path}')
        rosys.notify(f'Driving path {path.name}...', 'info')
        self.PATH_DRIVING_STARTED.emit(path.path)
        await self.driver.drive_to(path.path[0].spline.start)
        await self.driver.drive_path(path.path)
        self.state = 'idle'
        rosys.notify(f'Path {path.name} succesfully driven', 'positive')
        # self.driver.parameters.can_drive_backwards = False

    async def get_reference(self) -> Optional[tuple[float, float]]:
        if self.gnss.device is None:
            self.log.warning('not getting reference because no GNSS device found')
            return None
        self.log.info('getting reference')
        rosys.notify('Waiting for GNSS fixed reference position...', 'info')
        start_time = rosys.time()
        while rosys.time() - start_time < 10:
            reference_lat = self.gnss.get_current_lat()
            reference_lon = self.gnss.get_current_lon()
            if reference_lat is not None and reference_lon is not None:
                rosys.notify('GNSS fixed reference position found', 'positive')
                return reference_lat, reference_lon
            await rosys.sleep(1.0)
        rosys.notify('GNSS fixed reference position not found', 'negative')
        return None
