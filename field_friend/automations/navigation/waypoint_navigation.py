from __future__ import annotations

import gc
import logging
from abc import abstractmethod
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

from ..entity_locator import EntityLocator
from ..implements.implement import Implement
from ..implements.weeding_implement import WeedingImplement

if TYPE_CHECKING:
    from ...system import System


WAYPOINTS = [Point(x=3.0 * x, y=x % 2) for x in range(1, 15)]


class WaypointNavigation(rosys.persistence.Persistable):
    LINEAR_SPEED_LIMIT: float = 0.13

    def __init__(self, system: System, implement: Implement) -> None:
        super().__init__()
        self.log = logging.getLogger('field_friend.navigation')
        self.system = system
        self.implement = implement

        self.detector = system.detector
        self.driver = system.driver
        self.gnss = system.gnss
        self.plant_provider = system.plant_provider
        self.plant_locator = system.plant_locator
        self.robot_locator = system.robot_locator

        self.name = 'Waypoint Navigation'
        self._upcoming_path: list[DriveSegment] = []
        self.linear_speed_limit = self.LINEAR_SPEED_LIMIT

        self.PATH_GENERATED = Event[list[DriveSegment]]()
        """a new path has been generated (argument: ``list[DriveSegment]``)"""

        self.SEGMENT_STARTED = Event[DriveSegment]()
        """a waypoint has been reached"""

        self.SEGMENT_COMPLETED = Event[DriveSegment]()
        """a waypoint has been reached"""

        self.SEGMENT_STARTED.register(self._handle_segment_started)

    @property
    def path(self) -> list[DriveSegment]:
        return self._upcoming_path

    @property
    def current_segment(self) -> DriveSegment | None:
        if not self._upcoming_path:
            return None
        return self._upcoming_path[0]

    @property
    def has_waypoints(self) -> bool:
        """Returns True as long as there are waypoints to drive to"""
        return self.current_segment is not None

    def _handle_segment_started(self, segment: DriveSegment) -> None:
        if isinstance(self.detector, rosys.vision.DetectorSimulation) and not rosys.is_test:
            self.detector.simulated_objects.clear()
            if self.plant_provider is not None:
                self.plant_provider.clear()
            if segment.use_implement:
                self.create_segment_simulation(segment)
        if isinstance(self.plant_locator, EntityLocator):
            if segment.use_implement:
                self.plant_locator.resume()
            else:
                self.plant_locator.pause()

    @track
    async def prepare(self) -> bool:
        """Prepares the navigation for the start of the automation

        Returns true if all preparations were successful, otherwise false."""
        if self.plant_provider is not None:
            self.log.debug('Clearing plant provider')
            self.plant_provider.clear()
        if isinstance(self.detector, rosys.vision.DetectorSimulation) and not rosys.is_test:
            self.detector.simulated_objects = []
        self._upcoming_path = self.generate_path()
        if not self._upcoming_path:
            self.log.error('Path generation failed')
            return False
        self.PATH_GENERATED.emit(self._upcoming_path)
        return True

    @abstractmethod
    def generate_path(self) -> list[DriveSegment]:
        raise NotImplementedError('Subclasses must implement this method')

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
            rosys.notify('Automation started')
            self.log.debug('Navigation started')

            assert self.current_segment is not None
            self.SEGMENT_STARTED.emit(self.current_segment)
            while self.has_waypoints:
                await self._run()
                await rosys.sleep(0.1)
            rosys.notify('Automation finished', 'positive')
        except WorkflowException as e:
            rosys.notify(f'Automation failed: {e}', 'negative')
        finally:
            await self.implement.finish()
            await self.finish()
            await self.implement.deactivate()
            await self.driver.wheels.stop()

    async def _run(self) -> None:
        if not await self._get_valid_implement_target():
            self.log.debug('No move target found, continuing...')
            await rosys.automation.parallelize(
                self._drive_along_segment(linear_speed_limit=self.linear_speed_limit),
                self._block_until_implement_has_target(),
                return_when_first_completed=True,
            )
        if not self.has_waypoints:
            return
        assert self.current_segment is not None
        if isinstance(self.implement, WeedingImplement) and self.current_segment.use_implement:
            implement_target = await self._get_valid_implement_target()
            if not implement_target:
                self.log.debug('Implement has no target anymore. Possibly overshot, continuing...')
                return
            if not await self._follow_segment_until(implement_target):
                assert isinstance(self.detector, rosys.vision.Detector)
                await self.detector.NEW_DETECTIONS.emitted(5)
                return
            await self.driver.wheels.stop()
            self.implement.has_plants_to_handle()
            await self.implement.start_workflow()
            await self.implement.stop_workflow()

    @track
    async def finish(self) -> None:
        """Executed after the navigation is done"""
        self.log.debug('Navigation finished')
        gc.collect()  # NOTE: auto garbage collection is deactivated to avoid hiccups from Global Interpreter Lock (GIL) so we collect here to reduce memory pressure

    @track
    async def _drive_along_segment(self, *, linear_speed_limit: float = 0.3) -> None:
        """Drive the robot to the next waypoint of the navigation"""
        segment = self.current_segment
        if segment is None:
            return
        stop_at_end = segment.stop_at_end or len(self._upcoming_path) == 1
        with self.driver.parameters.set(linear_speed_limit=linear_speed_limit, can_drive_backwards=segment.backward):
            await self.driver.drive_spline(segment.spline, flip_hook=segment.backward, throttle_at_end=stop_at_end, stop_at_end=stop_at_end)
        self.SEGMENT_COMPLETED.emit(segment)
        self._upcoming_path.pop(0)
        if self.has_waypoints:
            assert self.current_segment is not None
            self.SEGMENT_STARTED.emit(self.current_segment)

    async def _block_until_implement_has_target(self) -> Point:
        while True:
            assert isinstance(self.current_segment, DriveSegment)
            if (target := await self._get_valid_implement_target()):
                return target
            await rosys.sleep(0.1)

    def _remove_segments_behind_robot(self, path_segments: list[DriveSegment]) -> list[DriveSegment]:
        """Create new path (list of segments) starting at the closest segment to the current pose"""
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
    async def _follow_segment_until(self, target: Point) -> bool:
        """Drives to a target point along the current spline.

        :param target: The target point to drive to
        """
        current_segment = self.current_segment
        if current_segment is None:
            return False
        current_pose = self.robot_locator.pose
        spline = current_segment.spline
        current_t = spline.closest_point(current_pose.x, current_pose.y)
        target_t = spline.closest_point(target.x, target.y, t_min=-0.2, t_max=1.2)
        work_x_corrected_pose = self._target_pose_on_current_segment(target)
        distance_to_target = current_pose.distance(work_x_corrected_pose)
        target_t = spline.closest_point(work_x_corrected_pose.x, work_x_corrected_pose.y, t_min=-0.2, t_max=1.2)
        if abs(distance_to_target) < self.driver.parameters.minimum_drive_distance:
            # TODO: quickfix for weeds behind the robot
            self.log.debug('Target close, working with out advancing... (%.6f m)', distance_to_target)
            return True
        if target_t < current_t or target_t > 1.0:
            # TODO: we need a sturdy function to advance a certain distance on a spline, because this method is off by a tiny amount. That's why +0.00003
            # test_weeding.py::test_advance_when_target_behind_robot tests this case. The weed is skipped in this case
            advance_distance = self.driver.parameters.minimum_drive_distance
            while True:
                target_pose = current_pose + PoseStep(linear=advance_distance, angular=0.0, time=0.0)
                target_t = spline.closest_point(target_pose.x, target_pose.y)
                advance_spline = sub_spline(spline, current_t, target_t)
                if advance_spline.estimated_length() > self.driver.parameters.minimum_drive_distance:
                    break
                advance_distance += 0.00001
            self.log.debug('Target behind robot, continue for %.6f meters', advance_distance)
            with self.driver.parameters.set(linear_speed_limit=self.linear_speed_limit):
                await self.driver.drive_spline(advance_spline, throttle_at_end=False, stop_at_end=False)
            return False
        self.log.debug('Driving to %s from target %s', work_x_corrected_pose, target)
        target_spline = sub_spline(spline, current_t, target_t)
        with self.driver.parameters.set(linear_speed_limit=self.linear_speed_limit):
            await self.driver.drive_spline(target_spline)
        return True

    def _target_pose_on_current_segment(self, target: Point) -> Pose:
        assert self.current_segment is not None
        spline = self.current_segment.spline
        target_t = spline.closest_point(target.x, target.y, t_min=-0.2, t_max=1.2)
        target_pose = spline.pose(target_t)
        return target_pose + PoseStep(linear=-self.system.field_friend.WORK_X, angular=0, time=0)

    async def _get_valid_implement_target(self) -> Point | None:
        if self.current_segment is None or not self.current_segment.use_implement:
            return None
        implement_target = await self.implement.get_target()
        if not implement_target:
            return None
        t = self.current_segment.spline.closest_point(implement_target.x, implement_target.y)
        if t in (0.0, 1.0):
            self.log.debug('Target is on segment end, continuing...')
            return None
        work_x_corrected_pose = self._target_pose_on_current_segment(implement_target)
        distance_to_target = self.robot_locator.pose.distance(work_x_corrected_pose)
        t = self.current_segment.spline.closest_point(work_x_corrected_pose.x, work_x_corrected_pose.y)
        if t in (0.0, 1.0) and abs(distance_to_target) > self.driver.parameters.minimum_drive_distance:
            # TODO: quickfix for weeds behind the robot
            self.log.debug('WorkX corrected target is on segment end, continuing...')
            return None
        return implement_target

    def backup_to_dict(self) -> dict[str, Any]:
        return {
            'linear_speed_limit': self.linear_speed_limit,
        }

    def restore_from_dict(self, data: dict[str, Any]) -> None:
        self.linear_speed_limit = data.get('linear_speed_limit', self.linear_speed_limit)

    def create_segment_simulation(self, segment: DriveSegment, *, first_plant_distance: float = 0.3, crop_distance: float = 0.3) -> None:
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
class DriveSegment(rosys.driving.PathSegment):
    # TODO: move methods to rosys.driving.PathSegment
    use_implement: bool = False
    stop_at_end: bool = True

    @property
    def start(self) -> Pose:
        return self.spline.pose(t=0)

    @property
    def end(self) -> Pose:
        return self.spline.pose(t=1)

    @classmethod
    def from_poses(cls, start: Pose, end: Pose, *, use_implement: bool = False, backward: bool = False, stop_at_end: bool = True) -> Self:
        return cls(spline=Spline.from_poses(start, end, backward=backward), use_implement=use_implement, backward=backward, stop_at_end=stop_at_end)

    @classmethod
    def from_points(cls, start: Point, end: Point, *, use_implement: bool = False, backward: bool = False, stop_at_end: bool = True) -> Self:
        yaw = start.direction(end)
        start_pose = Pose(x=start.x, y=start.y, yaw=yaw)
        end_pose = Pose(x=end.x, y=end.y, yaw=yaw)
        return cls.from_poses(start_pose, end_pose, use_implement=use_implement, backward=backward, stop_at_end=stop_at_end)


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
