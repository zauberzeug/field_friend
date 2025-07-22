from __future__ import annotations

import gc
import logging
from dataclasses import dataclass
from random import randint
from typing import TYPE_CHECKING, Any, Self

import numpy as np
import rosys
import rosys.geometry
from nicegui import ui
from rosys.analysis import track
from rosys.event import Event
from rosys.geometry import GeoReference, Point, Point3d, Pose, PoseStep, Spline
from rosys.hardware import Gnss

from ..implements.implement import Implement

if TYPE_CHECKING:
    from ...automations.implements.implement import Implement
    from ...system import System


WAYPOINTS = [Point(x=3.0 * x, y=x % 2) for x in range(1, 10)]


class Navigation(rosys.persistence.Persistable):
    LINEAR_SPEED_LIMIT: float = 0.13

    def __init__(self, system: System, implement: Implement) -> None:
        super().__init__()
        self.log = logging.getLogger('field_friend.navigation')
        self.system = system
        self.implement = implement
        self.driver = system.driver
        self.robot_locator = system.robot_locator
        self.gnss = system.gnss
        self.plant_provider = system.plant_provider
        self.puncher = system.puncher
        self.detector = system.detector

        self.name = 'Waypoint Navigation'
        self._upcoming_path: list[PathSegment | WorkingSegment] = []
        self._use_implement = True
        self.start_position = self.robot_locator.pose.point
        self.linear_speed_limit = self.LINEAR_SPEED_LIMIT

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

    @track
    async def start(self) -> None:
        try:
            if not is_reference_valid(self.gnss):
                rosys.notify('GNSS not available or reference too far away', 'warning')
                await rosys.sleep(3)
            if not await self.prepare():
                self.log.error('Preparation failed')
                return
            if not await self.implement.prepare():
                self.log.error('Implement preparation failed')
                return
            await self.implement.activate()
            self.start_position = self.robot_locator.pose.point
            rosys.notify('Automation started')
            self.log.debug('Navigation started')

            async def get_nearest_target() -> Point:
                while True:
                    move_target = await self.implement.get_move_target()
                    if move_target and self._use_implement:
                        return move_target
                    await rosys.sleep(0.1)

            while not self._should_finish():
                await rosys.automation.parallelize(
                    self._drive(),
                    get_nearest_target(),
                    return_when_first_completed=True,
                )
                while not self._should_finish():
                    move_target = await self.implement.get_move_target()
                    if not move_target:
                        self.log.debug('No move target found, continuing...')
                        break
                    if not await self.drive_towards_target(move_target):
                        await self.implement.get_move_target()
                        await rosys.sleep(0.1)
                        continue
                    await self.implement.get_move_target()
                    await self.implement.start_workflow()
                    await self.implement.stop_workflow()
                    await rosys.sleep(0.1)
                await rosys.sleep(0.1)
            rosys.notify('Automation finished', 'positive')
        except WorkflowException as e:
            rosys.notify(f'Automation failed: {e}', 'negative')
        finally:
            await self.implement.finish()
            await self.finish()
            await self.implement.deactivate()
            await self.driver.wheels.stop()

    @track
    async def prepare(self) -> bool:
        """Prepares the navigation for the start of the automation

        Returns true if all preparations were successful, otherwise false."""
        self.plant_provider.clear()
        if isinstance(self.detector, rosys.vision.DetectorSimulation) and not rosys.is_test:
            self.detector.simulated_objects = []
        self.log.info('clearing plant provider')
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
    async def finish(self) -> None:
        """Executed after the navigation is done"""
        self.log.debug('Navigation finished')
        gc.collect()

    @track
    async def _drive(self) -> None:
        """Drive the robot to the next waypoint of the navigation"""
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
    async def drive_towards_target(self, target: Point, **kwargs) -> bool:
        """Drives the robot towards a target point, but keeps the robot on a set heading with a limited turn angle.

        :param target: The target point to drive towards
        """
        current_segment = self.current_segment
        if current_segment is None:
            return False
        current_pose = self.robot_locator.pose
        spline = current_segment.spline
        current_t = spline.closest_point(current_pose.x, current_pose.y)
        target_t = spline.closest_point(target.x, target.y, t_min=-0.2, t_max=1.2)
        target_pose = spline.pose(target_t)
        work_x_corrected_pose = target_pose + PoseStep(linear=-self.system.field_friend.WORK_X, angular=0, time=0)
        target_t = spline.closest_point(work_x_corrected_pose.x, work_x_corrected_pose.y, t_min=-0.2, t_max=1.2)
        if current_t >= target_t:
            # TODO: inspect why the target is sometimes behind the robot on the spline
            self.log.warning('Target behind robot, continue for %s meters',
                             self.driver.parameters.minimum_drive_distance)
            step = PoseStep(linear=self.driver.parameters.minimum_drive_distance, angular=0.0, time=0.0)
            target_pose = current_pose + step
            target_t = spline.closest_point(target_pose.x, target_pose.y)
            new_spline = sub_spline(spline, current_t, target_t)
            with self.driver.parameters.set(linear_speed_limit=self.linear_speed_limit):
                await self.driver.drive_spline(new_spline, flip_hook=False, throttle_at_end=True, stop_at_end=False)
            return False
        self.log.warning('Driving to %s from target %s', target_pose, target)
        new_spline = sub_spline(spline, current_t, target_t)
        with self.driver.parameters.set(linear_speed_limit=self.linear_speed_limit):
            await self.driver.drive_spline(new_spline, flip_hook=False, throttle_at_end=True, stop_at_end=True)
        return True

    @track
    async def turn_to_yaw(self, target_yaw: float, *, angle_threshold: float = np.deg2rad(1.0)) -> None:
        """Turns the robot on the spot to a target heading.

        :param target_yaw: The target heading in radians
        :param angle_threshold: The accepted threshold around the target heading in radians, default is 1 degree
        """
        # TODO: growing error because of the threshold
        while True:
            angle = rosys.helpers.eliminate_2pi(target_yaw - self.robot_locator.pose.yaw)
            if abs(angle) < angle_threshold:
                break
            sign = 1 if angle > 0 else -1
            angular = 0.5 / self.driver.parameters.minimum_turning_radius * sign
            await self.driver.wheels.drive(*self.driver._throttle(0.0, angular))  # pylint: disable=protected-access
            await rosys.sleep(0.1)
        await self.driver.wheels.stop()

    def _should_finish(self) -> bool:
        """Returns True if the navigation should stop and be finished"""
        return self.current_segment is None

    def backup_to_dict(self) -> dict[str, Any]:
        return {
            'linear_speed_limit': self.linear_speed_limit,
        }

    def restore_from_dict(self, data: dict[str, Any]) -> None:
        self.linear_speed_limit = data.get('linear_speed_limit', self.linear_speed_limit)

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

    def settings_ui(self) -> None:
        ui.number('Linear Speed',
                  step=0.01,
                  min=self.driver.parameters.throttle_at_end_min_speed,
                  max=self.driver.parameters.linear_speed_limit,
                  format='%.2f',
                  suffix='m/s',
                  on_change=self.request_backup) \
            .props('dense outlined') \
            .classes('w-24') \
            .bind_value(self, 'linear_speed_limit') \
            .tooltip(f'Forward speed limit between {self.driver.parameters.throttle_at_end_min_speed} and {self.driver.parameters.linear_speed_limit} m/s (default: {self.LINEAR_SPEED_LIMIT:.2f})')

    def developer_ui(self) -> None:
        pass


class WorkflowException(Exception):
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


def is_reference_valid(gnss: Gnss | None, *, max_distance: float = 5000.0) -> bool:
    if gnss is None:
        return True
    if GeoReference.current is None:
        return False
    if gnss.last_measurement is None:
        return False
    if gnss.last_measurement.gps_quality == 0:
        return False
    return gnss.last_measurement.point.distance(GeoReference.current.origin) <= max_distance


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
