
import logging
from typing import Any, Optional

import numpy as np
import rosys
from rosys.geometry import Pose, Spline
from rosys.helpers import angle
from shapely.geometry import LineString

from ..hardware import FieldFriend
from ..navigation import Field, FieldProvider, Gnss
from .coverage_planer import CoveragePlanner
from .sequence import find_sequence


class Mowing:

    def __init__(self, field_friend: FieldFriend, field_provider: FieldProvider, driver: rosys.driving.Driver,
                 path_planner: rosys.pathplanning.PathPlanner, gnss: Gnss, *, robot_width: float) -> None:
        self.log = logging.getLogger('field_friend.path_recorder')
        self.field_friend = field_friend
        self.field_provider = field_provider
        self.driver = driver
        self.path_planner = path_planner
        self.gnss = gnss
        self.coverage_planner = CoveragePlanner(self)

        self.padding: float = 1.0
        self.lane_distance: float = 0.5
        self.turning_radius: float = self.driver.parameters.minimum_turning_radius
        self.robot_width: float = robot_width

        self.field: Optional[Field] = None
        self.paths: Optional[list[list[rosys.driving.PathSegment]]] = None
        self.current_path: Optional[list[rosys.driving.PathSegment]] = None
        self.current_path_segment: Optional[rosys.driving.PathSegment] = None
        self.continue_mowing: bool = False

        self.MOWING_STARTED = rosys.event.Event()
        """Mowing has started."""

        self.needs_backup = False
        rosys.persistence.register(self)

    def backup(self) -> dict:
        return {
            'padding': self.padding,
            'lane_distance': self.lane_distance,
            'paths': [[rosys.persistence.to_dict(segment) for segment in path] for path in self.paths],
            'current_path': [rosys.persistence.to_dict(segment) for segment in self.current_path],
            'current_path_segment': rosys.persistence.to_dict(self.current_path_segment) if self.current_path_segment else None,
        }

    def restore(self, data: dict[str, Any]) -> None:
        self.padding = data.get('padding', self.padding)
        self.lane_distance = data.get('lane_distance', self.lane_distance)
        paths_data = data.get('paths', [])
        self.paths = [
            [rosys.persistence.from_dict(rosys.driving.PathSegment, segment_data) for segment_data in path_data]
            for path_data in paths_data
        ]
        current_path_data = data.get('current_path', [])
        self.current_path = [rosys.persistence.from_dict(
            rosys.driving.PathSegment, segment_data) for segment_data in current_path_data]
        self.current_path_segment = rosys.persistence.from_dict(
            rosys.driving.PathSegment, data['current_path_segment']) if data['current_path_segment'] else None

    def invalidate(self) -> None:
        self.needs_backup = True

    async def start(self) -> None:
        self.log.info('starting mowing')
        if self.field_friend.estop.active or self.field_friend.estop.is_soft_estop_active:
            rosys.notify('E-Stop is active', 'negative')
            return
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
        if self.padding < self.robot_width:
            self.padding = self.robot_width
        await self._mowing()

    async def _mowing(self) -> None:
        rosys.notify('Starting mowing')
        while True:
            try:
                if not self.continue_mowing:
                    self.path_planner.obstacles.clear()
                    self.path_planner.areas.clear()
                    for obstacle in self.field.obstacles:
                        self.path_planner.obstacles[obstacle.id] = rosys.pathplanning.Obstacle(
                            id=obstacle.id, outline=obstacle.points)
                    area = rosys.pathplanning.Area(id=f'{self.field.id}', outline=self.field.outline)
                    self.path_planner.areas = {area.id: area}
                    self.paths = self._generate_mowing_path()
                    self.invalidate()
                    if not self.paths:
                        rosys.notify('No paths to drive', 'negative')
                        raise Exception('No paths to drive')
                self.MOWING_STARTED.emit([path_segment for path in self.paths for path_segment in path])
                await self._drive_mowing_paths(self.paths)
                rosys.notify('Mowing finished', 'positive')
                break
            except Exception as e:
                self.log.exception(e)
                rosys.notify(f'Mowing failed because of {e}', 'negative')
                break

    def _generate_mowing_path(self) -> list[list[rosys.driving.PathSegment]]:
        self.log.info('generating mowing path')
        lane_groups, outer_lanes_groups = self.coverage_planner.decompose_into_lanes()
        paths = []
        outer_lanes = [lane for lane_group in outer_lanes_groups for lane in lane_group]
        splines = []
        for lane in outer_lanes:
            lane_points = lane.coords
            p1_start = rosys.geometry.Point(x=lane_points[0][0], y=lane_points[0][1])
            p1_end = rosys.geometry.Point(x=lane_points[1][0], y=lane_points[1][1])
            yaw = p1_start.direction(p1_end)
            distance = self.turning_radius * 2
            if distance >= p1_start.distance(p1_end):
                distance = p1_start.distance(p1_end)/3
            p1_start = p1_start.polar(distance, yaw)
            p1_end = p1_end.polar(distance, yaw-np.pi)
            splines.append(
                rosys.geometry.Spline.from_poses(
                    rosys.geometry.Pose(x=p1_start.x, y=p1_start.y, yaw=yaw),
                    rosys.geometry.Pose(x=p1_end.x, y=p1_end.y, yaw=yaw)))
        splines_with_turns = self._generate_turn_splines(splines)
        splines = splines_with_turns

        path = [rosys.driving.PathSegment(spline=spline) for spline in splines]
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
        if self.continue_mowing:
            first_path = self.current_path
        else:
            first_path = paths[0]
        await self.driver.drive_to(first_path[0].spline.start)
        for path in paths:
            if self.continue_mowing and path != self.current_path:
                continue
            self.current_path = path
            self.invalidate()
            if path != first_path:
                start_pose = self.driver.odometer.prediction
                end_pose = path[0].spline.pose(0)
                path_switch = await self.path_planner.search(start=start_pose, goal=end_pose, timeout=30)
                if path_switch is None:
                    self.log.warning('not driving because no path to start point found')
                    raise Exception('no path to start point found')
                self.driver.parameters.can_drive_backwards = True
                self.driver.parameters.hook_offset = 1.2
                self.driver.parameters.carrot_distance = 0.4
                self.driver.parameters.carrot_offset = self.driver.parameters.hook_offset + self.driver.parameters.carrot_distance
                await self.driver.drive_path(path_switch)
                self.driver.parameters.can_drive_backwards = False
                self.driver.parameters.hook_offset = 0.6
                self.driver.parameters.carrot_distance = 0.2
                self.driver.parameters.carrot_offset = self.driver.parameters.hook_offset + self.driver.parameters.carrot_distance
            for segment in path:
                if self.continue_mowing:
                    if segment == self.current_path_segment:
                        self.continue_mowing = False
                    else:
                        continue
                self.current_path_segment = segment
                self.invalidate()
                await self.driver.drive_spline(segment.spline, throttle_at_end=segment == path[-1], flip_hook=segment.backward)
        self.paths = []
        self.current_path = None
        self.current_path_segment = None
        self.invalidate()

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
                self.log.warning('!!!no valid sequence found!!!')
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

    def _generate_turn_splines(self, lanes: list[rosys.geometry.Spline]) -> list[rosys.geometry.Spline]:
        splines = []
        for i in range(len(lanes)):
            splines.append(lanes[i])
            if i != len(lanes) - 1:
                lane_angle = angle(lanes[i].yaw(1), lanes[i + 1].yaw(0))
                if lane_angle < np.pi / 4 and lane_angle > -np.pi / 4:
                    self.log.info(f'no turn needed at spline {np.pi/2} {lane_angle}')
                    continue

                middle_point = lanes[i].end.polar(
                    self.turning_radius, lanes[i].pose(1).yaw).polar(
                    self.turning_radius, lanes[i].end.direction(lanes[i + 1].start))
                middle_pose = Pose(x=middle_point.x, y=middle_point.y,
                                   yaw=lanes[i].end.direction(
                                       lanes[i+1].start))
                first_turn_spline = rosys.geometry.Spline.from_poses(
                    Pose(
                        x=lanes[i].end.x, y=lanes[i].end.y,
                        yaw=lanes[i].start.direction(
                            lanes[i].end)),
                    middle_pose)
                splines.append(first_turn_spline)
                second_turn_spline = rosys.geometry.Spline.from_poses(
                    middle_pose,
                    Pose(
                        x=lanes[i+1].start.x, y=lanes[i+1].start.y,
                        yaw=lanes[i+1].start.direction(
                            lanes[i+1].end)))
                splines.append(second_turn_spline)
        return splines
