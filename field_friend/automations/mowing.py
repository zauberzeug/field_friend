
import logging
from itertools import chain
from typing import Optional

import numpy as np
import rosys
from rosys.geometry import Pose, Spline
from shapely import affinity
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

        self.padding: float = 1.0
        self.lane_distance: float = 0.5
        self.corner_radius: float = self.driver.parameters.minimum_turning_radius

        self.field: Optional[Field] = None
        self.MOWING_STARTED = rosys.event.Event()
        """Mowing has started."""

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
            if not distance or distance > 10:
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
                if self.corner_radius * 2 > self.lane_distance:
                    self.minimum_distance = int(np.ceil(self.corner_radius * 2 / self.lane_distance))
                else:
                    self.minimum_distance = 1
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
        lane_groups = self._decompose_into_lanes()
        paths = []
        for lanes in lane_groups:
            odered_splines = self._make_plan(lanes)
            splines = self._generate_turn_splines(odered_splines)
            path = [rosys.driving.PathSegment(spline=spline) for spline in splines]
            if path:
                paths.append(path)
        return paths

    async def _drive_mowing_paths(self, paths: list[list[rosys.driving.PathSegment]]) -> None:
        self.driver.parameters.can_drive_backwards = True
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
            await self.driver.drive_path(path_switch)
            await self.driver.drive_path(path)
        self.driver.parameters.can_drive_backwards = False

    def _decompose_into_lanes(self) -> list[LineString]:
        self.log.info('decomposing field into lanes')
        lanes_groups = []  # List to hold lists of lanes
        current_groups = [[]]  # Current groups of lanes
        is_multiline = False

        min_x = self.field.outline[0].x
        min_y = self.field.outline[0].y
        translated_field = Polygon([(point.x - min_x, point.y - min_y) for point in self.field.outline])

        # Determine the unit vector direction of the first line of the polygon
        p1, p2 = translated_field.exterior.coords[:2]
        direction = np.array([p2[0] - p1[0], p2[1] - p1[1]])
        direction /= np.linalg.norm(direction)

        # Calculate the angle of rotation based on the direction
        theta = np.arctan2(direction[1], direction[0])

        # Rotate the translated_field
        rotated_field = affinity.rotate(translated_field, -theta, origin=(0, 0), use_radians=True)

        padded_polygon = rotated_field.buffer((-self.padding - self.corner_radius)*self.minimum_distance)

        obstacle_polygons = []  # List to hold all obstacle polygons
        if self.field.obstacles:  # Check if the obstacles list is not empty
            for obstacle in self.field.obstacles:
                translated_obstacle = Polygon([(point.x - min_x, point.y - min_y) for point in obstacle.points])
                rotated_obstacle = affinity.rotate(translated_obstacle, -theta, origin=(0, 0), use_radians=True)
                padded_obstacle = rotated_obstacle.buffer(self.padding + self.corner_radius)
                obstacle_polygons.append(padded_obstacle)
        obstacles_union = unary_union(obstacle_polygons) if obstacle_polygons else None

        if obstacles_union is not None:
            navigable_area = padded_polygon.difference(obstacles_union)
        else:
            navigable_area = padded_polygon

        # Direction and perpendicular direction vectors after rotation
        direction = np.array([1, 0])
        perp_direction = np.array([0, 1])

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
                    if not current_groups or len(current_groups) != len(intersection.geoms):
                        for group in current_groups:
                            lanes_groups.append(group)
                        current_groups = [[] for _ in range(len(intersection.geoms))]
                    for i, line_segment in enumerate(intersection.geoms):
                        current_groups[i].append(line_segment)
                    is_multiline = True
                elif isinstance(intersection, LineString):
                    if is_multiline:
                        for group in current_groups:
                            lanes_groups.append(group)
                        current_groups = [[]]  # Create new group for LineString
                        is_multiline = False
                    current_groups[0].append(intersection)

            length += self.lane_distance

        for group in current_groups:
            lanes_groups.append(group)

        lanes_groups = [
            [affinity.translate(  # Translating the lanes back to their original position
                affinity.rotate(  # Rotating the lanes back to their original orientation
                    line,
                    theta,
                    origin=(0, 0),
                    use_radians=True
                ),
                xoff=min_x,
                yoff=min_y
            ) for line in group]
            for group in lanes_groups
        ]
        return lanes_groups

    def _make_plan(self, lanes: list[LineString]) -> list[Spline]:
        self.log.info(f'converting {len(lanes)} lanes into splines and finding sequence')

        if self.minimum_distance > 1:
            sequence = find_sequence(len(lanes), minimum_distance=self.minimum_distance)
        else:
            sequence = list(range(len(lanes)))
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
