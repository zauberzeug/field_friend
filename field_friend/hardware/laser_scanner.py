from __future__ import annotations

import logging
import os
import subprocess
from dataclasses import dataclass

import numpy as np
import rosys
from nicegui import ui
from rosys.geometry import Point

logging.getLogger().setLevel(logging.INFO)


@dataclass
class LaserScan:
    time: float
    points: list[Point]


@dataclass
class Cylinder:
    point: Point
    radius: float


def calculate_distances(scan: Scan) -> list[float]:
    distances = []
    zero_point = Point(x=0, y=0)
    for p in scan.points:
        dist = zero_point.distance(p)
        distances.append(dist)
    return distances


def calculate_angles(scan: Scan) -> list[float]:
    angles = []
    zero_point = Point(x=0, y=0)
    for point in scan.points:
        angle = zero_point.direction(point)
        angles.append(angle)
    return angles


def calculate_derivatives(distances: list[float], min_distance: float = 0.01, max_distance: float = 10.0) -> list[float]:
    derivatives = []
    for i in range(1, len(distances) - 1):
        previous_value = distances[i-1]
        next_value = distances[i+1]
        if previous_value < min_distance or next_value < min_distance:
            derivatives.append(0.0)
            continue
        if previous_value > max_distance or next_value > max_distance:
            derivatives.append(0.0)
            continue

        derivative = (next_value - previous_value) / 2
        derivatives.append(derivative)
    derivatives = [0.0, *derivatives, 0.0]
    return derivatives


def find_cylinder(scan: Scan, min_depth_change: float = 0.02, min_pole_radius: float = 0.03, max_pole_radius: float = 0.3) -> list[Cylinder]:
    if len(scan.points) < 3:
        logging.debug(f'Too few points: {len(scan.points)}')
        return []

    angles = calculate_angles(scan)
    distances = calculate_distances(scan)
    logging.debug(f'Processing scan with {len(distances)} points')

    derivatives = calculate_derivatives(distances)
    logging.debug(f'Derivative range: min={min(derivatives):.3f}, max={max(derivatives):.3f}')

    cylinders: list[Cylinder] = []
    zero_point = Point(x=0, y=0)
    start: int | None = None
    for i, derivative in enumerate(derivatives):
        # logging.info(f"Processing index {i} (derivative: {derivative:.3f}) with distance={distances[i]}")
        if derivative <= -min_depth_change:
            start = i
            logging.info(f'Found potential cylinder start at index {i} (derivative: {derivative:.3f})')
            continue
        if start is None:
            continue

        if derivative < min_depth_change:
            # logging.info(f"Rejecting segment: derivative {i} - {derivative:.3f} is too small")
            continue

        number_of_points = i - start
        if number_of_points <= 3:
            # logging.info(f"Rejecting segment: too few points ({number_of_points})")
            start = None
            continue

        start_angle = angles[start]
        end_angle = angles[i]
        center_angle = 0.5 * (start_angle + end_angle)
        half_angle = 0.5 * (end_angle - start_angle)
        logging.info(f'Analyzing segment: start_idx={start}, end_idx={i}, points={number_of_points}')
        logging.info(
            f'Angles: start={np.rad2deg(start_angle):.1f}°, end={np.rad2deg(end_angle):.1f}°, span={np.rad2deg(end_angle-start_angle):.1f}°')

        distance = 0.0
        center_count = 0
        number_of_points += 1
        for j in range(start + number_of_points // 3, i - number_of_points // 3 + 1):
            d = distances[j]
            if 0.01 < d < 1.0:
                distance += d
                center_count += 1

        if center_count > 0:
            distance /= center_count
            radius = (distance * np.sin(half_angle)) / (1.0 - np.sin(half_angle))
            logging.info(f'Candidate cylinder: distance={distance:.3f}m, radius={radius:.3f}m')

            if min_pole_radius <= radius <= max_pole_radius:
                center_distance = distance + radius
                cylinder = zero_point.polar(center_distance, center_angle)
                cylinders.append(Cylinder(point=cylinder, radius=radius))
                logging.info(f'Found valid cylinder at ({cylinder.x:.3f}, {cylinder.y:.3f})')
            else:
                logging.info(
                    f'Rejecting cylinder: radius {radius:.3f}m outside valid range [{min_pole_radius}, {max_pole_radius}]')
        else:
            logging.info(f'No valid cylinder found at index {i} with start={start}')

        start = None

    logging.info(f'Found {len(cylinders)} cylinders')
    return cylinders


class LaserScanner:
    def __init__(self, *, svg_size: tuple[int, int] = (360, 360), svg_scale: float = 250.0) -> None:
        self.log = logging.getLogger('laser_scanner')
        self.last_scan: LaserScan | None = None
        self.is_active: bool = False

        self.svg_size = svg_size
        self.svg_center = (self.svg_size[0] / 2, self.svg_size[1] / 2)
        self.svg_scale = svg_scale
        self._map_content = ''
        self.base_svg = f'<svg viewBox="0 0 {self.svg_size[0]} {self.svg_size[1]}" xmlns="http://www.w3.org/2000/svg">'
        self.base_svg += f'<circle cx="{self.svg_center[0]}" cy="{self.svg_center[1]}" \
            r="{3}" fill="red"/>'
        self.last_draw_timestamp: float = 0.0

        self.NEW_SCAN = rosys.event.Event()
        '''a new scan is available (argument: ndarray with scan points in world coordinate system)'''


class LaserScannerHardware(LaserScanner):
    SEPARATOR = '---\n'
    # --channel --serial /dev/ttyUSB0 115200

    def __init__(self, serial_port: str) -> None:
        super().__init__()

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
                    self.last_scan = scan
                    self.NEW_SCAN.emit(scan)
                except ValueError as e:
                    self.log.warning('Could not parse lidar data:')
                    self.log.warning(e)

        if self.process is not None and not self.is_active:
            await rosys.run.io_bound(self.stop)

    def _process_data(self, data: np.ndarray) -> LaserScan:
        now = rosys.time()
        zero_point = Point(x=0, y=0)
        points: list[Point] = []
        data = data[data[:, 1].argsort()]
        for quality, angle, distance in data:
            if quality == 0 or distance == 0:
                continue
            point = zero_point.polar(distance / 1000.0, np.deg2rad(angle))
            points.append(point)
        return LaserScan(time=now, points=points)

    def start(self) -> None:
        self.log.warning('turning lidar on')
        executable = '/usr/local/bin/rplidar'
        command = [executable, '--channel', '--serial', self.serial_port, '115200']
        self.process = subprocess.Popen(command, stdout=subprocess.PIPE)
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
    NUM_POINTS: int = 100
    DISTANCE: float = 5.0

    def __init__(self) -> None:
        super().__init__()
        rosys.on_repeat(self.simulate_scan, 0.1)

    async def simulate_scan(self) -> None:
        zero_point = Point(x=0, y=0)
        distances: list[float] = [np.random.uniform(
            self.DISTANCE * (1.0 - self.NOISE), self.DISTANCE * (1.0 + self.NOISE)) for _ in range(self.NUM_POINTS)]
        angles: list[float] = [np.deg2rad(i * 360 / self.NUM_POINTS) for i in range(self.NUM_POINTS)]
        points = [zero_point.polar(distance, angle) for distance, angle in zip(distances, angles, strict=True)]
        scan = LaserScan(time=rosys.time(), points=points)
        self.last_scan = scan
        self.NEW_SCAN.emit(scan)


class LaserScanMap(ui.interactive_image):
    def __init__(self, scale: float = 250.0, size: tuple[int, int] = (360, 360)) -> None:
        super().__init__(size=size)
        self.size = size
        self.scale = scale
        self.center = (self.size[0] / 2, self.size[1] / 2)
        self.base_svg = f'<svg viewBox="0 0 {self.size[0]} {self.size[1]}" xmlns="http://www.w3.org/2000/svg"><rect width="100%" height="100%" fill="#f0f0f0"/>'
        self.base_svg += f'<circle cx="{self.center[0]}" cy="{self.center[1]}" \
            r="{3}" fill="red"/>'
        self.last_update: float = 0.0
        rosys.on_startup(lambda: self.draw(''))

    def draw(self, svg_content: str):
        svg = self.base_svg
        svg += svg_content
        svg += '</svg>'
        self.last_update = rosys.time()
        self.content = svg

    def generate_scan_svg(self, scan: LaserScan) -> str:
        svg = ''
        for point in scan.points:
            x = int(self.center[0] + self.scale * point.x)
            y = int(self.center[1] + self.scale * point.y)
            svg += f'<circle cx="{x}" cy="{y}" r="1" fill="blue"/>'
        return svg
