from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np
import rosys
from nicegui import ui
from rosys.geometry import Point

from ...hardware import LaserScan, LaserScanMap
from .locator import Locator

if TYPE_CHECKING:
    from ...system import System


@dataclass
class Tree:
    point: Point
    radius: float


class TreeLocator(Locator):
    def __init__(self, system: System) -> None:
        super().__init__(system)
        # self.laser_scanner: LaserScanner = system.laser_scanner
        # self.laser_scanner.NEW_SCAN.register(self.on_new_scan)
        self._last_scan: LaserScan | None = None
        self._update_scan: bool = True
        self._map_update_interval: float = 0.5

        self.NEW_TREES = rosys.event.Event()
        """a new scan is available (argument: list of trees)"""

    async def on_new_scan(self, scan: LaserScan) -> None:
        self._last_scan = scan
        trees = self.find_cylinder(scan)
        self.NEW_TREES.emit(trees)

    # def backup(self) -> dict:
    #     return super().backup() | {}

    # def restore(self, data: dict[str, Any]) -> None:
    #     super().restore(data)

    def find_cylinder(self, scan: LaserScan, min_depth_change: float = 0.02, min_pole_radius: float = 0.03, max_pole_radius: float = 0.3) -> list[Tree]:
        if len(scan.points) < 3:
            self.log.debug(f'Too few points: {len(scan.points)}')
            return []

        angles = calculate_angles(scan)
        distances = calculate_distances(scan)
        self.log.debug(f'Processing scan with {len(distances)} points')

        derivatives = calculate_derivatives(distances)
        self.log.debug(f'Derivative range: min={min(derivatives):.3f}, max={max(derivatives):.3f}')

        cylinders: list[Tree] = []
        zero_point = Point(x=0, y=0)
        start: int | None = None
        for i, derivative in enumerate(derivatives):
            # logging.info(f"Processing index {i} (derivative: {derivative:.3f}) with distance={distances[i]}")
            if derivative <= -min_depth_change:
                start = i
                self.log.debug(f'Found potential cylinder start at index {i} (derivative: {derivative:.3f})')
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
            self.log.debug(f'Analyzing segment: start_idx={start}, end_idx={i}, points={number_of_points}')
            self.log.debug(
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
                self.log.debug(f'Candidate cylinder: distance={distance:.3f}m, radius={radius:.3f}m')

                if min_pole_radius <= radius <= max_pole_radius:
                    center_distance = distance + radius
                    cylinder = zero_point.polar(center_distance, center_angle)
                    cylinders.append(Tree(point=cylinder, radius=radius))
                    self.log.debug(f'Found valid cylinder at ({cylinder.x:.3f}, {cylinder.y:.3f})')
                else:
                    self.log.debug(
                        f'Rejecting cylinder: radius {radius:.3f}m outside valid range [{min_pole_radius}, {max_pole_radius}]')
            else:
                self.log.debug(f'No valid cylinder found at index {i} with start={start}')

            start = None

            self.log.debug(f'Found {len(cylinders)} cylinders')
        return cylinders

    def developer_ui(self) -> None:
        ui.label('Tree Locator').classes('text-center text-bold')
        super().developer_ui()
        ui.checkbox('Update scan').bind_value(self, '_update_scan')
        scan_map = LaserScanMap()
        scan_map.classes('w-[32rem]')

        def draw_scan(trees: list[Tree]) -> None:
            if not self._update_scan:
                return
            if self._last_scan is None:
                return
            if rosys.time() - scan_map.last_update < self._map_update_interval:
                return
            svg = ''
            svg += scan_map.generate_scan_svg(self._last_scan)
            for tree in trees:
                x = int(scan_map.center[0] + scan_map.scale * tree.point.x)
                y = int(scan_map.center[1] + scan_map.scale * tree.point.y)
                svg += f'<circle cx="{x}" cy="{y}" r="3" fill="green"/>'
                svg += f'<circle cx="{x}" cy="{y}" r="{tree.radius * scan_map.scale}" \
                    stroke="green" stroke-width="1" fill="none" opacity="0.3"/>'
            scan_map.draw(svg)
        self.NEW_TREES.register(draw_scan)


def calculate_angles(scan: LaserScan) -> list[float]:
    angles = []
    zero_point = Point(x=0, y=0)
    for point in scan.points:
        angle = zero_point.direction(point)
        angles.append(angle)
    return angles


def calculate_distances(scan: LaserScan) -> list[float]:
    distances = []
    zero_point = Point(x=0, y=0)
    for p in scan.points:
        dist = zero_point.distance(p)
        distances.append(dist)
    return distances


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
