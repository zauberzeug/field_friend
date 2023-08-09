
import logging
from typing import Any, Optional

import numpy as np
import rosys
from rosys.geometry import Pose, Spline
from shapely import affinity
from shapely.geometry import LineString, MultiLineString, Polygon
from shapely.ops import unary_union

from ..navigation import Field, FieldProvider, Gnss
from .coverage_planer import CoveragePlanner
from .sequence import find_sequence


class Mowing:

    def __init__(self, field_provider: FieldProvider, driver: rosys.driving.Driver,
                 path_planner: rosys.pathplanning.PathPlanner, gnss: Gnss) -> None:
        self.log = logging.getLogger('field_friend.path_recorder')
        self.field_provider = field_provider
        self.driver = driver
        self.path_planner = path_planner
        self.gnss = gnss
        self.coverage_planner = CoveragePlanner(self)

        self.padding: float = 1.0
        self.lane_distance: float = 0.5
        self.turning_radius: float = self.driver.parameters.minimum_turning_radius

        self.field: Optional[Field] = None
        self.MOWING_STARTED = rosys.event.Event()
        """Mowing has started."""

        self.needs_backup = False
        rosys.persistence.register(self)

    def backup(self) -> dict:
        return {
            'padding': rosys.persistence.to_dict(self.padding),
            'lane_distance': rosys.persistence.to_dict(self.lane_distance),
        }

    def restore(self, data: dict[str, Any]) -> None:
        self.padding = data.get('padding', self.padding)
        self.lane_distance = data.get('lane_distance', self.lane_distance)

    async def start(self) -> None:
        self.log.info('starting mowing')
        if self.gnss.device is None:
            rosys.notify('No GNSS device found', 'negative')
            return
        if self.gnss.device != 'simulation':
            if self.field.reference_lat is None or self.field.reference_lon is None:
                rosys.notify('Field has no reference location set', 'negative')
                return
            self.gnss.set_reference(self.field.reference_lat, self.field.reference_lon)
            distance = self.gnss.calculate_distance(self.gnss.record.latitude, self.gnss.record.longitude)
            if not distance or distance > 50:
                rosys.notify('Distance to reference location is too large', 'negative')
                return
        if self.field is None and self.field_provider.fields:
            self.field = self.field_provider.fields[0]
        if len(self.field.outline) < 3:
            rosys.notify('No field is defined', 'negative')
            return
        await self._mowing()

    async def _mowing(self) -> None:
        rosys.notify('Starting mowing')
        while True:
            try:
                self.path_planner.obstacles.clear()
                self.path_planner.areas.clear()
                for obstacle in self.field.obstacles:
                    self.path_planner.obstacles[obstacle.name] = rosys.pathplanning.Obstacle(id=obstacle.name,
                                                                                             outline=obstacle.points)
                area = rosys.pathplanning.Area(id=f'{self.field.name}', outline=self.field.outline)
                self.path_planner.areas = {area.id: area}
                paths = self._generate_mowing_path()
                self.MOWING_STARTED.emit([path_segment for path in paths for path_segment in path])
                await self._drive_mowing_paths(paths)
                rosys.notify('Mowing finished', 'positive')
                break
            except Exception as e:
                self.log.exception(e)
                rosys.notify('Mowing failed', 'negative')
                break

    def _generate_mowing_path(self) -> list[list[rosys.driving.PathSegment]]:
        self.log.info('generating mowing path')
        lane_groups, outer_lanes_groups = self.coverage_planner.decompose_into_lanes()
        paths = []
        for outer_lanes in outer_lanes_groups:
            splines = []
            for lane in outer_lanes:
                lane_points = lane.coords
                p1 = rosys.geometry.Point(x=lane_points[0][0], y=lane_points[0][1])
                p2 = rosys.geometry.Point(x=lane_points[1][0], y=lane_points[1][1])
                yaw = p1.direction(p2)
                shorten_p1 = p1.polar(self.turning_radius, yaw)
                shorten_p2 = p2.polar(self.turning_radius, yaw-np.pi)
                splines.append(
                    rosys.geometry.Spline.from_poses(
                        rosys.geometry.Pose(x=shorten_p1.x, y=shorten_p1.y, yaw=yaw),
                        rosys.geometry.Pose(x=shorten_p2.x, y=shorten_p2.y, yaw=yaw)))
            connected_splines = self._generate_turn_splines(splines)
            path = [rosys.driving.PathSegment(spline=spline) for spline in connected_splines]
            if path:
                paths.append(path)

        for lanes in lane_groups:
            odered_splines = self._make_plan(lanes)
            splines = self._generate_turn_splines(odered_splines)
            path = [rosys.driving.PathSegment(spline=spline) for spline in splines]
            if path:
                paths.append(path)

        return paths

    async def _drive_mowing_paths(self, paths: list[list[rosys.driving.PathSegment]]) -> None:
        if not paths:
            raise Exception('no paths to drive')
        first_path = paths.pop(0)
        await self.driver.drive_to(first_path[0].spline.start)
        await self.driver.drive_path(first_path)
        for path in paths:
            start_pose = self.driver.odometer.prediction
            end_pose = path[0].spline.pose(0)
            self.log.info(f'end pose: {end_pose}')
            path_switch = await self.path_planner.search(start=start_pose, goal=end_pose, timeout=20)
            self.log.info(f'start path: {path_switch}')
            if path_switch is None:
                self.log.warning('not driving because no path to start point found')
                return
            self.driver.parameters.can_drive_backwards = True
            await self.driver.drive_path(path_switch)
            self.driver.parameters.can_drive_backwards = False
            await self.driver.drive_path(path)

    def _make_plan(self, lanes: list[LineString]) -> list[Spline]:
        self.log.info(f'converting {len(lanes)} lanes into splines and finding sequence')
        if self.turning_radius * 2 > self.lane_distance:
            minimum_distance = int(np.ceil(self.turning_radius * 2 / self.lane_distance))
        else:
            minimum_distance = 1
        self.log.info(f'minimum distance: {minimum_distance}')
        if minimum_distance > 1:
            sequence = find_sequence(len(lanes), minimum_distance=minimum_distance)
            if sequence == []:  # ToDO: handle missing sequence better
                self.log.warning('no valid sequence found')
                sequence = list(range(len(lanes)))
        else:
            sequence = list(range(len(lanes)))
        self.log.info(f'sequence of splines: {sequence}')
        splines = []
        for i, index in enumerate(sequence):
            lane = lanes[index]
            lane_points = lane.coords
            if i % 2 != 0:
                lane_points = lane_points[::-1]
            lane_splines = []
            for i in range(len(lane_points) - 1):
                p1, p2 = lane_points[i], lane_points[i + 1]
                yaw = np.arctan2(p2[1] - p1[1], p2[0] - p1[0])
                lane_splines.append(rosys.geometry.Spline.from_poses(rosys.geometry.Pose(
                    x=p1[0], y=p1[1], yaw=yaw), rosys.geometry.Pose(x=p2[0], y=p2[1], yaw=yaw)))
            splines.append(*lane_splines)
        return splines

    def _generate_turn_splines(self, rows: list[rosys.geometry.Spline]) -> list[rosys.geometry.Spline]:
        splines = []
        for i in range(len(rows)):
            splines.append(rows[i])
            if i != len(rows) - 1:
                turn_spline = rosys.geometry.Spline.from_poses(
                    Pose(
                        x=rows[i].end.x, y=rows[i].end.y,
                        yaw=rows[i].start.direction(
                            rows[i].end)),
                    Pose(
                        x=rows[i+1].start.x, y=rows[i+1].start.y,
                        yaw=rows[i+1].start.direction(
                            rows[i+1].end)),)
                splines.append(turn_spline)
        return splines
