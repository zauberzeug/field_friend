from __future__ import annotations

from dataclasses import dataclass
from random import randint
from typing import TYPE_CHECKING, Self

import numpy as np
import rosys
from rosys.analysis import track
from rosys.event import Event
from rosys.geometry import Point, Point3d, Pose, PoseStep, Spline

from .navigation import Navigation

if TYPE_CHECKING:
    from ...automations.implements.implement import Implement
    from ...system import System


WAYPOINTS = [Point(x=3.0 * x, y=x % 2) for x in range(1, 10)]


class WaypointNavigation(Navigation):
    def __init__(self, system: System, implement: Implement) -> None:
        super().__init__(system, implement)
        self.name = 'Waypoint Navigation'
        self._upcoming_path: list[PathSegment | WorkingSegment] = []

        self.PATH_GENERATED = Event[list[PathSegment | WorkingSegment]]()
        """a new path has been generated (argument: ``list[PathSegment | WorkingSegment]``)"""

        self.WAYPOINT_REACHED = Event[[]]()
        """a waypoint has been reached"""

    @property
    def path(self) -> list[PathSegment | WorkingSegment]:
        return self._upcoming_path

    @property
    def current_segment(self) -> PathSegment | WorkingSegment | None:
        if not self._upcoming_path:
            return None
        return self._upcoming_path[0]

    @property
    def target(self) -> Pose | None:
        if self.current_segment is None:
            return None
        return self.current_segment.end

    @property
    def target_heading(self) -> float:
        if self.current_segment is None:
            return self.robot_locator.pose.yaw
        current_pose = self.robot_locator.pose
        # TODO: estimated_length is costly, it should be cached
        look_ahead_t = 0.1 / self.current_segment.spline.estimated_length()
        t = self.current_segment.spline.closest_point(current_pose.x, current_pose.y)
        # TODO: test if this can cause issues, when t = 1.0
        spline_pose = self.current_segment.spline.pose(min(1.0, t + look_ahead_t))
        return current_pose.direction(spline_pose.point)

    def generate_path(self):
        last_pose = Pose(x=0, y=0, yaw=0)
        path: list[PathSegment | WorkingSegment] = []
        segment: PathSegment | WorkingSegment
        for waypoint in WAYPOINTS:
            next_pose = Pose(x=waypoint.x, y=waypoint.y, yaw=last_pose.yaw)
            if next_pose.y % 2:
                segment = WorkingSegment.from_poses(last_pose, next_pose, stop_at_end=False)
            else:
                segment = PathSegment.from_poses(last_pose, next_pose, stop_at_end=False)
            path.append(segment)
            last_pose = next_pose
        path = self._start_at_closest_segment(path)
        return path

    def _start_at_closest_segment(self, path_segments: list[PathSegment | WorkingSegment]) -> list[PathSegment | WorkingSegment]:
        """Filter path segments to start at the closest segment to the current pose"""
        current_pose = self.robot_locator.pose
        start_index = 0
        for i, segment in enumerate(path_segments):
            t = segment.spline.closest_point(current_pose.x, current_pose.y, t_min=-0.1, t_max=1.1)
            if t > 0.99:
                continue
            start_index = i
            break
        return path_segments[start_index:]

    @track
    async def prepare(self) -> bool:
        await super().prepare()
        self._upcoming_path = self.generate_path()
        if not self._upcoming_path:
            self.log.error('Path generation failed')
            return False
        self.PATH_GENERATED.emit(self._upcoming_path)
        if self._should_finish():
            self.log.error('Preparation failed - should finish')
            return False
        return True

    @track
    async def _drive(self) -> None:
        while not self._should_finish():
            segment = self.current_segment
            if segment is None:
                return
            if isinstance(self.detector, rosys.vision.DetectorSimulation) and not rosys.is_test:
                self.detector.simulated_objects.clear()
                self.system.plant_provider.clear()
                if isinstance(segment, WorkingSegment):
                    self.create_segment_simulation(segment)
            stop_at_end = segment.stop_at_end or len(self._upcoming_path) == 1
            self._use_implement = isinstance(segment, WorkingSegment)
            with self.driver.parameters.set(linear_speed_limit=self.linear_speed_limit, can_drive_backwards=segment.backward):
                await self.driver.drive_spline(segment.spline, flip_hook=segment.backward, throttle_at_end=stop_at_end, stop_at_end=stop_at_end)
            self._upcoming_path.pop(0)
            self.WAYPOINT_REACHED.emit()

    @track
    async def drive_towards_target(self, target: Point | Pose, **kwargs) -> None:
        """Drives the robot towards a target point, but keeps the robot on a set heading with a limited turn angle.

        :param target: The target point to drive towards
        """
        current_segment = self.current_segment
        if current_segment is None:
            return
        current_pose = self.robot_locator.pose
        spline = current_segment.spline
        current_t = spline.closest_point(current_pose.x, current_pose.y)
        target_t = spline.closest_point(target.x, target.y, t_min=-0.2, t_max=1.2)
        target_in_front = current_t < target_t
        stop_at_end = True
        if target_in_front:
            target_pose = spline.pose(target_t)
            self.log.debug('Driving to %s from target %s', target_pose, target)
            new_spline = sub_spline(spline, current_t, target_t)
        else:
            # TODO: inspect why the target is sometimes behind the robot on the spline
            self.log.debug('Target behind robot, continue for %s meters', self.driver.parameters.minimum_drive_distance)
            step = PoseStep(linear=self.driver.parameters.minimum_drive_distance, angular=0.0, time=0.0)
            target_pose = current_pose + step
            target_t = spline.closest_point(target_pose.x, target_pose.y)
            new_spline = sub_spline(spline, current_t, target_t)
            stop_at_end = False
        with self.driver.parameters.set(linear_speed_limit=self.linear_speed_limit, can_drive_backwards=False):
            await self.driver.drive_spline(new_spline, flip_hook=False, throttle_at_end=True, stop_at_end=stop_at_end)

    def _should_finish(self) -> bool:
        return self.current_segment is None

    def create_segment_simulation(self, segment: WorkingSegment, *, first_plant_distance: float = 0.3, crop_distance: float = 0.3) -> None:
        detector = self.system.detector
        if not isinstance(detector, rosys.vision.DetectorSimulation):
            return
        spline = segment.spline
        spline_length = spline.estimated_length()
        if spline_length < first_plant_distance:
            return
        start_t = first_plant_distance / spline_length
        number_of_crops = round((spline_length - first_plant_distance) / crop_distance)
        for crop_t in np.linspace(start_t, 1.0, number_of_crops):
            crop_pose = segment.spline.pose(crop_t)
            crop_point = crop_pose.point.polar(randint(-2, 2)*0.01, crop_pose.yaw+np.pi/2)
            detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='sugar_beet',
                                                                           position=Point3d(x=crop_point.x, y=crop_point.y, z=0)))
            for _ in range(1, 7):
                weed_point = crop_point.polar(randint(-5, 5)*0.01, crop_pose.yaw) \
                    .polar(randint(-15, 15)*0.01, crop_pose.yaw + np.pi/2)
                detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='weed',
                                                                               position=Point3d(x=weed_point.x, y=weed_point.y, z=0)))

    def developer_ui(self) -> None:
        pass


@dataclass(slots=True, kw_only=True)
class PathSegment(rosys.driving.PathSegment):
    stop_at_end: bool = True

    @property
    def start(self) -> Pose:
        return self.spline.pose(t=0)

    @property
    def end(self) -> Pose:
        return self.spline.pose(t=1)

    @classmethod
    def from_poses(cls, start: Pose, end: Pose, *, backward: bool = False, stop_at_end: bool = True) -> Self:
        return cls(spline=Spline.from_poses(start, end, backward=backward), backward=backward, stop_at_end=stop_at_end)

    @classmethod
    def from_points(cls, start: Point, end: Point, *, backward: bool = False, stop_at_end: bool = True) -> Self:
        yaw = start.direction(end)
        start_pose = Pose(x=start.x, y=start.y, yaw=yaw)
        end_pose = Pose(x=end.x, y=end.y, yaw=yaw)
        return cls.from_poses(start_pose, end_pose, backward=backward, stop_at_end=stop_at_end)


@dataclass(slots=True, kw_only=True)
class WorkingSegment(PathSegment):
    ...


def sub_spline(spline: Spline, t_min: float, t_max: float) -> Spline:
    """Creates a new spline from a sub-segment of the given spline"""
    # TODO: move to rosys.geometry.spline
    def split_cubic(p0: Point, p1: Point, p2: Point, p3: Point, t: float) -> tuple[tuple[Point, Point, Point, Point], tuple[Point, Point, Point, Point]]:
        """Split a cubic Bezier at t, returns left and right as (start, c1, c2, end)"""
        q0 = p0.interpolate(p1, t)
        q1 = p1.interpolate(p2, t)
        q2 = p2.interpolate(p3, t)
        r0 = q0.interpolate(q1, t)
        r1 = q1.interpolate(q2, t)
        s0 = r0.interpolate(r1, t)
        return (p0, q0, r0, s0), (s0, r1, q2, p3)

    P0, P1, P2, P3 = spline.start, spline.control1, spline.control2, spline.end
    _, (Q0, Q1, Q2, Q3) = split_cubic(P0, P1, P2, P3, t_min)
    s = (t_max - t_min) / (1 - t_min) if t_min != 1 else 0.0
    (R0, R1, R2, R3), _ = split_cubic(Q0, Q1, Q2, Q3, s)
    return Spline(start=R0, control1=R1, control2=R2, end=R3)
