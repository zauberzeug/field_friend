from __future__ import annotations

import logging
import os
import subprocess
from dataclasses import dataclass

import nicegui
import numpy as np
import rosys
from rosys.geometry import Point, Point3d, Pose3d


@dataclass(slots=True, kw_only=True)
class LaserScan:
    time: float
    distances: list[float]
    angles: list[float]

    @property
    def points(self) -> list[Point]:
        return self.angles_and_distances_to_points(self.angles, self.distances)

    def transformed_points(self, scanner_pose: Pose3d) -> list[Point]:
        return [Point3d.from_point(point).transform_with(scanner_pose).projection() for point in self.angles_and_distances_to_points(self.angles, self.distances)]

    @staticmethod
    def angles_and_distances_to_points(angles: list[float], distances: list[float]) -> list[Point]:
        assert len(angles) == len(distances), f'Length mismatch: angles={len(angles)}, distances={len(distances)}'
        zero_point = Point(x=0, y=0)
        return [zero_point.polar(distance, angle) for distance, angle in zip(distances, angles, strict=True)]


class LaserScanner:
    def __init__(self, *, pose: Pose3d | None = None) -> None:
        self.pose = pose or Pose3d.zero()
        self.pose = self.pose.as_frame('laser_scanner')
        self.log = logging.getLogger('laser_scanner')
        self.last_scan: LaserScan | None = None
        self.is_active: bool = False

        self.NEW_SCAN = rosys.event.Event()
        '''a new scan is available (argument: LaserScan)'''

    def emit_laser_scan(self, scan: LaserScan) -> None:
        self.last_scan = scan
        self.NEW_SCAN.emit(scan)


class LaserScannerHardware(LaserScanner):
    SEPARATOR = '---\n'
    # --channel --serial /dev/ttyUSB0 115200

    def __init__(self, serial_port: str, **kwargs) -> None:
        super().__init__(**kwargs)

        self.serial_port = serial_port
        self.process: subprocess.Popen | None = None
        self.buffer: str = ''
        self.is_active = True

        rosys.on_startup(self.start)
        rosys.on_shutdown(self.tear_down)
        rosys.on_repeat(self.step, 0.02)

    async def step(self) -> None:
        if self.process is None and self.is_active:
            await rosys.run.io_bound(self.start)

        if self.process is not None:
            assert self.process.stdout is not None
            new_data = (self.process.stdout.read() or b'').decode()
            # self.log.warning(f'reading from process: {new_data}')
            self.buffer += new_data
            if self.SEPARATOR in new_data:
                data, self.buffer = self.buffer.split(self.SEPARATOR)[-2:]
                try:
                    data_array: np.ndarray = np.array([tuple(map(float, line.split())) for line in data.splitlines()])
                    if len(data_array) < 10:
                        return
                    scan = self._process_data(data_array)
                    if len(scan.points) < 10:
                        return
                    self.emit_laser_scan(scan)
                except ValueError as e:
                    self.log.warning('Could not parse lidar data:')
                    self.log.warning(e)

        if self.process is not None and not self.is_active:
            await rosys.run.io_bound(self.stop)

    def _process_data(self, data: np.ndarray) -> LaserScan:
        """The sensor returns clockwise angles, but we want counterclockwise angles for a right-handed coordinate system"""
        now = rosys.time()
        data = data[data[:, 1].argsort()]
        distances = np.where(data[:, 2] == 0, 500_000, data[:, 2]) / 1000.0
        angles = np.deg2rad(360 - data[:, 1])
        angles = np.flip(angles)
        distances = np.flip(distances)
        return LaserScan(time=now, distances=distances, angles=angles)

    def start(self) -> None:
        self.log.warning('turning lidar on')
        executable = '/usr/local/bin/rplidar'
        command = [executable, '--channel', '--serial', self.serial_port, '115200']
        self.process = subprocess.Popen(command, stdout=subprocess.PIPE)  # pylint: disable=consider-using-with
        assert self.process is not None
        assert self.process.stdout is not None
        os.set_blocking(self.process.stdout.fileno(), False)

    async def tear_down(self) -> None:
        await rosys.run.io_bound(self.stop)

    def stop(self) -> None:
        self.is_active = False
        if self.process is None:
            return
        self.log.info('turning lidar off')
        try:
            self.process.terminate()
        except Exception:
            self.log.exception('could not terminate process')
        self.process = None


class LaserScannerSimulation(LaserScanner):
    NOISE: float = 0.02
    NUM_POINTS: int = 50
    DISTANCE: float = 5.0
    INTERVAL: float = 0.5

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        rosys.on_repeat(self.simulate_scan, self.INTERVAL)

    async def simulate_scan(self) -> None:
        distances: list[float] = [np.random.uniform(self.DISTANCE * (1.0 - self.NOISE),
                                                    self.DISTANCE * (1.0 + self.NOISE)) for _ in range(self.NUM_POINTS)]
        angles: list[float] = [np.deg2rad(i * 360 / self.NUM_POINTS) for i in range(self.NUM_POINTS)]
        scan = LaserScan(time=rosys.time(), distances=distances, angles=angles)
        self.emit_laser_scan(scan)


class LaserScanObject(nicegui.elements.scene_objects.PointCloud):
    def __init__(self, laser_scanner: LaserScanner, *, is_active: bool = True, render_distance: float = 40.0, **kwargs) -> None:
        super().__init__(points=[], point_size=0.02, **kwargs)
        self.is_active = is_active
        self.laser_scanner = laser_scanner
        self.render_distance = render_distance
        rosys.on_repeat(self.update, 0.5)

    def update(self) -> None:
        if not self.is_active:
            self.set_points([])
            return
        scan = self.laser_scanner.last_scan
        if scan is None:
            return
        laser_scanner_pose = self.laser_scanner.pose.resolve()
        filtered_points = [transformed_point for transformed_point, distance in zip(scan.transformed_points(laser_scanner_pose), scan.distances, strict=True)
                           if distance < self.render_distance]
        self.set_points([[point.x, point.y, laser_scanner_pose.z] for point in filtered_points],
                        colors=[[0.0, 0.0, 1.0] for _ in filtered_points])
