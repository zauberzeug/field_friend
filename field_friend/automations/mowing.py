
import logging
from itertools import chain
from typing import Optional

import numpy as np
import rosys
from rosys.geometry import Pose, Spline
from shapely.geometry import LineString, MultiLineString, Polygon
from shapely.ops import unary_union

from ..navigation import Field, FieldProvider, Gnss
from .sequence import find_sequence


class Mowing:

    def __init__(self, field_provider: FieldProvider, driver: rosys.driving.Driver,
                 path_planner: rosys.pathplanning.PathPlanner, gnss: Gnss) -> None:
        self.log = logging.getLogger('field_friend.path_recorder')
        self.field_provider = field_provider
        self.driver = driver
        self.path_planner = path_planner
        self.gnss = gnss

        self.padding: float = 0.5
        self.lane_distance: float = 0.4
        self.corner_radius: float = self.driver.parameters.minimum_turning_radius
        self.minimum_distance = int(np.ceil(self.corner_radius * 2 / self.lane_distance))
        self.field: Optional[Field] = None
        self.MOWING_STARTED = rosys.event.Event()
        """Mowing has started."""

    async def start(self) -> None:
        self.log.info('starting mowing')
        rosys.notify('Starting mowing')
        if self.gnss.device is None:
            self.log.warning('not driving because no GNSS device found')
            return
        if self.gnss.device != 'simulation':
            if self.field.reference_lat is None or self.field.reference_lon is None:
                self.log.warning('not driving because no reference location set')
                return
            self.gnss.set_reference(self.field.reference_lat, self.field.reference_lon)
            distance = self.gnss.calculate_distance(self.gnss.record.latitude, self.gnss.record.longitude)
            if not distance or distance > 10:
                self.log.warning('not driving because distance to reference location is too large')
                rosys.notify('Distance to reference location is too large', 'negative')
                return
        self.field = self.field_provider.fields[0] if self.field_provider.fields else None
        if self.field is None or len(self.field.outline) < 3:
            self.log.warning(f'not driving because no field is defined field: {self.field}')
            return
        self.path_planner.obstacles.clear()
        self.path_planner.areas.clear()
        for obstacle in self.field.obstacles:
            if len(obstacle.points) < 3:
                self.log.warning(f'not driving because obstacle has less than 3 points: {obstacle}')
                continue
            self.path_planner.obstacles[obstacle.name] = rosys.pathplanning.Obstacle(id=obstacle.name,
                                                                                     outline=obstacle.points)
        self.log.info(f'obstacles: {self.path_planner.obstacles}')
        area = rosys.pathplanning.Area(id=f'{self.field.name}', outline=self.field.outline)
        self.path_planner.areas = {area.id: area}
        self.log.info(f'areas: {self.path_planner.areas}')
        paths = self.generate_mowing_path()
        self.MOWING_STARTED.emit(chain([path_segment for path in paths for path_segment in path]))
        self.driver.parameters.can_drive_backwards = True
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
            await self.driver.drive_path(path_switch)
            await self.driver.drive_path(path)
        rosys.notify('Mowing finished', 'positive')
        self.driver.parameters.can_drive_backwards = False

    def decompose_into_lanes(self) -> list[LineString]:
        self.log.info('decomposing field into lanes')
        lanes_groups = []  # List to hold lists of lanes
        current_groups = [[]]  # Current groups of lanes
        is_multiline = False

        self.polygon = Polygon([(point.x, point.y) for point in self.field.outline])
        padded_polygon = self.polygon.buffer(-self.padding - self.corner_radius)

        obstacle_polygons = []  # List to hold all obstacle polygons
        if self.field.obstacles:  # Check if the obstacles list is not empty
            for obstacle in self.field.obstacles:
                obstacle_polygon = Polygon([(point.x, point.y) for point in obstacle.points])
                padded_obstacle = obstacle_polygon.buffer(self.padding + self.corner_radius)
                obstacle_polygons.append(padded_obstacle)
        obstacles_union = unary_union(obstacle_polygons) if obstacle_polygons else None

        if obstacles_union is not None:
            navigable_area = padded_polygon.difference(obstacles_union)
        else:
            navigable_area = padded_polygon
        # Determine the unit vector direction of the first line of the polygon
        p1, p2 = self.polygon.exterior.coords[:2]
        direction = np.array([p2[0] - p1[0], p2[1] - p1[1]])
        direction /= np.linalg.norm(direction)

        # Determine the perpendicular direction
        perp_direction = np.array([-direction[1], direction[0]])

        # Determine the minimum and maximum projections along the direction and perpendicular direction
        min_proj = max_proj = np.dot(np.array(p1), direction)
        min_perp_proj = max_perp_proj = np.dot(np.array(p1), perp_direction)
        for coord in padded_polygon.exterior.coords:
            proj = np.dot(coord, direction)
            perp_proj = np.dot(coord, perp_direction)
            min_proj = min(min_proj, proj)
            max_proj = max(max_proj, proj)
            min_perp_proj = min(min_perp_proj, perp_proj)
            max_perp_proj = max(max_perp_proj, perp_proj)

        length = min_perp_proj
        while length <= max_perp_proj:
            start_point = np.array(p1) + length * perp_direction + min_proj * direction
            end_point = np.array(p1) + length * perp_direction + max_proj * direction
            line = LineString([start_point, end_point])

            # Intersect the line with the inverse of the obstacle polygon
            intersection = navigable_area.intersection(line)

            if intersection and not intersection.is_empty:
                if isinstance(intersection, MultiLineString):
                    self.log.info(f'Multiline found')
                    if not current_groups or len(current_groups) != len(intersection.geoms):
                        for group in current_groups:
                            lanes_groups.append(group)
                        current_groups = [[] for _ in range(len(intersection.geoms))]
                    for i, line_segment in enumerate(intersection.geoms):
                        current_groups[i].append(line_segment)
                    is_multiline = True
                elif isinstance(intersection, LineString):
                    self.log.info(f'LineString found')
                    if is_multiline:
                        for group in current_groups:
                            lanes_groups.append(group)
                        current_groups = [[]]  # Create new group for LineString
                        is_multiline = False
                    current_groups[0].append(intersection)

            length += self.lane_distance

        for group in current_groups:
            lanes_groups.append(group)
        self.log.info(f'lane groups: {len(lanes_groups)}')
        return lanes_groups

    def make_plan(self, lanes: list[LineString]) -> list[Spline]:
        self.log.info(f'converting {len(lanes)} lanes into splines and finding sequence')
        sequence = find_sequence(len(lanes), minimum_distance=self.minimum_distance)
        self.log.info(f'sequence of splines: {sequence}')
        splines = []

        for i, index in enumerate(sequence):
            lane = lanes[index]
            # Extract the points from the lane
            lane_points = lane.coords if isinstance(lane, LineString) else lane[0].coords

            # Reverse the points of every other lane
            if i % 2 != 0:
                lane_points = lane_points[::-1]

            # Create a new splines list for this lane
            lane_splines = []

            for i in range(len(lane_points) - 1):
                # Add straight segments
                p1, p2 = lane_points[i], lane_points[i + 1]
                yaw = np.arctan2(p2[1] - p1[1], p2[0] - p1[0])
                lane_splines.append(rosys.geometry.Spline.from_poses(rosys.geometry.Pose(
                    x=p1[0], y=p1[1], yaw=yaw), rosys.geometry.Pose(x=p2[0], y=p2[1], yaw=yaw)))

            splines.append(*lane_splines)

        return splines

    def generate_turn_splines(self, rows: list[rosys.geometry.Spline]) -> list[rosys.geometry.Spline]:
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

    def generate_mowing_path(self):
        self.log.info('generating mowing path')
        lane_groups = self.decompose_into_lanes()
        paths = []
        for lanes in lane_groups:
            odered_rows = self.make_plan(lanes)
            splines = self.generate_turn_splines(odered_rows)
            path = [rosys.driving.PathSegment(spline=spline) for spline in splines]
            if path:
                paths.append(path)

        return paths
