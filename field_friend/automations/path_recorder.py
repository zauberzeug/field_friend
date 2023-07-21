import logging
from typing import Any, Literal

import rosys
from rosys.driving import PathSegment
from rosys.geometry import Spline

from ..navigation import Gnss


class PathRecorder:
    def __init__(self, driver: rosys.driving.Driver, steerer: rosys.driving.Steerer, gnss: Gnss) -> None:
        self.log = logging.getLogger('field_friend.path_recorder')
        self.driver = driver
        self.steerer = steerer
        self.gnss = gnss
        self.PATHS_CHANGED = rosys.event.Event()
        """The dict of paths has changed."""

        self.PATH_DRIVING_STARTED = rosys.event.Event()
        """A path is being driven (argument: list of PathSegments)."""

        self.paths: dict[str, list[rosys.driving.PathSegment]] = {}
        self.current_path_recording: str = ''
        self.state: Literal['recording', 'driving', 'idle'] = 'idle'
        self.needs_backup: bool = False
        rosys.persistence.register(self)

    def backup(self) -> dict:
        paths_dict = {
            path_key: [rosys.persistence.to_dict(path_segment) for path_segment in path_value]
            for path_key, path_value in self.paths.items()
        }
        return {'paths': paths_dict}

    def restore(self, data: dict[str, Any]) -> None:
        paths_dict = data.get('paths', {})
        paths = {
            path_key: [rosys.persistence.from_dict(rosys.driving.PathSegment, path_segment_dict) for path_segment_dict in path_value]
            for path_key, path_value in paths_dict.items()
        }
        rosys.persistence.replace_dict(self.paths, list, paths)

    def invalidate(self) -> None:
        self.needs_backup = True
        self.PATHS_CHANGED.emit()

    async def record_path(self, name: str) -> None:
        # if self.gnss.device is None:
        #     self.log.warning('not recording because no GNSS device found')
        #     return
        # if self.gnss.reference_lat is None or self.gnss.reference_lon is None:
        #     self.log.warning('not recording because no reference location set')
        #     return
        if self.state != 'idle':
            self.log.warning(f'not recording because state is {self.state}')
            return
        self.log.info('recording path')
        self.current_path_recording = name
        self.state = 'recording'
        splines = []
        last_pose = self.driver.odometer.prediction
        while self.state == 'recording':
            if self.steerer.state != self.steerer.state.IDLE:
                new_pose = self.driver.odometer.prediction
                splines.append(Spline.from_poses(last_pose, new_pose))
                last_pose = new_pose
            else:
                self.log.info('not recording because no movement detected')
            await rosys.sleep(10)
        new_pose = self.driver.odometer.prediction
        splines.append(Spline.from_poses(last_pose, new_pose))
        self.paths[name] = [PathSegment(spline=spline) for spline in splines]
        self.log.info(f'path {name} recorded with {self.paths[name] if self.paths[name] else "NO"} segments')
        self.invalidate()

    async def drive_path(self, name: str) -> None:
        self.log.info(f'driving path {name}')
        # self.driver.parameters.can_drive_backwards = True
        # if self.gnss.device is None:
        #     self.log.warning('not driving because no GNSS device found')
        #     return
        if self.state != 'idle':
            self.log.warning(f'not driving because state is {self.state}')
            return
        self.state = 'driving'
        if name not in self.paths:
            self.log.warning(f'path {name} not found')
            return
        path = self.paths[name]
        if path == []:
            self.log.warning(f'path {name} is empty')
            return
        self.log.info(f'path: {path}')
        self.PATH_DRIVING_STARTED.emit(path)
        # await self.driver.drive_to(path[0].spline.start)
        await self.driver.drive_path(path)
        self.state = 'idle'
        self.driver.parameters.can_drive_backwards = False
