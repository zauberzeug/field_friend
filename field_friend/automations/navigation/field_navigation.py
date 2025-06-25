import gc
import math
from enum import Enum, auto
from random import randint
from typing import TYPE_CHECKING, Any

import numpy as np
import rosys
from nicegui import ui
from rosys.analysis import track
from rosys.geometry import Point

from ..computed_field import ComputedField, Row
from ..field_description import RowSupportPoint
from ..implements.implement import Implement
from ..implements.weeding_implement import WeedingImplement
from .navigation import WorkflowException, is_reference_valid
from .straight_line_navigation import StraightLineNavigation

if TYPE_CHECKING:
    from ...system import System


class State(Enum):
    APPROACH_START_ROW = auto()
    CHANGE_ROW = auto()
    FOLLOW_ROW = auto()
    WAITING_FOR_CONFIRMATION = auto()
    ROW_COMPLETED = auto()
    FIELD_COMPLETED = auto()
    ERROR = auto()


class FieldNavigation(StraightLineNavigation):
    MAX_DISTANCE_DEVIATION = 0.05
    MAX_ANGLE_DEVIATION = np.deg2rad(10.0)

    def __init__(self, system: 'System', implement: Implement) -> None:
        super().__init__(system, implement)
        self.name = 'Field Navigation'
        self.gnss = system.gnss
        self.bms = system.field_friend.bms
        self.automator = system.automator
        self.automation_watcher = system.automation_watcher
        self.field_provider = system.field_provider

        self._state = State.APPROACH_START_ROW
        self.row_index = 0
        self.force_first_row_start: bool = False
        self.is_in_swarm: bool = False
        self.allowed_to_turn: bool = False
        self.wait_distance: float = 1.3

        self.field: ComputedField | None = None
        self.field_id = self.field_provider.selected_field.source.id if self.field_provider.selected_field else None
        self.field_provider.FIELD_SELECTED.register(self._set_field_id)
        self._loop: bool = False
        self.rows_to_work_on: list[Row] = []

    @property
    def target_heading(self) -> float:
        assert self.field is not None
        assert self.start_point is not None
        assert self.end_point is not None
        return self.start_point.direction(self.end_point)

    @property
    def current_row(self) -> Row | None:
        if self.field is None:
            return None
        if len(self.rows_to_work_on) == 0:
            self.log.warning('No rows to work on')
            return None
        return self.rows_to_work_on[self.row_index]

    @track
    async def start(self) -> None:
        if not is_reference_valid(self.gnss):
            raise WorkflowException('reference to far away from robot')
        await super().start()

    async def prepare(self) -> bool:
        await super().prepare()
        self.field = self.field_provider.get_field(self.field_id)
        if self.field is None:
            rosys.notify('No field selected', 'negative')
            return False
        self.rows_to_work_on = self.field_provider.get_rows_to_work_on()
        if not self.rows_to_work_on or len(self.rows_to_work_on) == 0:
            rosys.notify('No rows available', 'negative')
            return False
        if self.gnss is None or not self.gnss.is_connected:
            rosys.notify('GNSS is not available', 'negative')
            return False
        for idx, row in enumerate(self.rows_to_work_on):
            if not len(row.points) >= 2:
                rosys.notify(f'Row {idx} on field {self.field.source.name} has not enough points', 'negative')
                return False
        nearest_row = self.get_nearest_row()
        if nearest_row is None:
            return False
        self._state = State.APPROACH_START_ROW
        self.plant_provider.clear()
        self.automation_watcher.gnss_watch_active = True
        self.automation_watcher.start_field_watch(self.field.outline)
        return True

    def _should_finish(self) -> bool:
        return self._state in (State.FIELD_COMPLETED, State.ERROR)

    async def finish(self) -> None:
        await super().finish()
        self.automation_watcher.gnss_watch_active = False
        self.automation_watcher.stop_field_watch()
        await self.implement.deactivate()

    def get_nearest_row(self) -> Row | None:
        assert self.field is not None
        assert self.gnss is not None
        assert self.gnss.is_connected
        if self.force_first_row_start:
            self.row_index = 0
        else:
            row = min(self.rows_to_work_on,
                      key=lambda r: r.line_segment().line.foot_point(self.robot_locator.pose.point).distance(self.robot_locator.pose.point))
            self.log.debug(f'Nearest row is {row.name}')
            self.row_index = self.rows_to_work_on.index(row)
        return self.rows_to_work_on[self.row_index]

    def get_upcoming_waypoints(self) -> Point | None:
        robot_pose = self.robot_locator.pose.point
        waypoints = [point.to_local() for point in self.current_row.points]
        segments = [(waypoints[i], waypoints[i+1]) for i in range(len(waypoints) - 1)]
        best_matching_target: Point | None = None
        best_matching_start: Point | None = None
        shortest_distance = float('inf')
        heading_threshold_rad = math.radians(30)

        # Try to match a segment normally
        for start_point, end_point in segments:
            segment_dx = end_point.x - start_point.x
            segment_dy = end_point.y - start_point.y
            segment_length_squared = segment_dx ** 2 + segment_dy ** 2

            if segment_length_squared == 0:
                continue  # Skip zero-length segments

            projection_ratio = (
                (robot_pose.x - start_point.x) * segment_dx +
                (robot_pose.y - start_point.y) * segment_dy
            ) / segment_length_squared

            if 0 <= projection_ratio <= 1:
                projection_point = Point(
                    x=start_point.x + projection_ratio * segment_dx,
                    y=start_point.y + projection_ratio * segment_dy
                )
                distance_to_projection = robot_pose.point.distance(projection_point)

                # Remove the heading constraint from segment matching
                if distance_to_projection < shortest_distance:
                    shortest_distance = distance_to_projection
                    best_matching_target = end_point
                    best_matching_start = start_point

        if best_matching_target and best_matching_start:
            # Find the indices of the start and end points
            try:
                start_index = waypoints.index(best_matching_start)
                end_index = waypoints.index(best_matching_target)

                # Determine direction based on robot's heading relative to segment direction
                segment_heading = best_matching_start.direction(best_matching_target)
                heading_difference = rosys.helpers.angle(robot_pose.yaw, segment_heading)

                # Debug output
                self.log.warning(
                    f'Robot heading: {math.degrees(robot_pose.yaw):.1f}°, Segment heading: {math.degrees(segment_heading):.1f}°, Difference: {math.degrees(heading_difference):.1f}°')

                # If heading difference is small, we're going forward; if large, we're going backward
                if abs(heading_difference) <= heading_threshold_rad:
                    # Forward direction: return waypoints from end point forward
                    self.log.warning(f'Going forward, returning waypoints from index {end_index}')
                    return waypoints[end_index:]
                else:
                    # Backward direction: return waypoints from start point backward
                    self.log.warning(f'Going backward, returning waypoints from index {start_index} backward')
                    return waypoints[start_index::-1]
            except ValueError:
                pass

        # No match on any segment — fallback to start or end point
        start = waypoints[0]
        end = waypoints[-1]

        dist_to_start = robot_pose.point.distance(start)
        dist_to_end = robot_pose.point.distance(end)

        angle_to_start = robot_pose.point.direction(start)
        angle_to_end = robot_pose.point.direction(end)

        angle_diff_to_start = rosys.helpers.angle(robot_pose.yaw, angle_to_start)
        angle_diff_to_end = rosys.helpers.angle(robot_pose.yaw, angle_to_end)

        # Check which endpoint is closer *and* roughly in front
        in_front_of_start = abs(angle_diff_to_start) <= heading_threshold_rad
        in_front_of_end = abs(angle_diff_to_end) <= heading_threshold_rad

        if in_front_of_start and (dist_to_start <= dist_to_end or not in_front_of_end):
            return waypoints  # Return all waypoints if starting from beginning
        if in_front_of_end:
            return [end]  # Return only the end point if it's the closest

        return []  # Return empty list if neither endpoint is in front

    def get_target(self) -> rosys.geometry.Pose | None:
        upcoming_waypoints: list[Point] = self.get_upcoming_waypoints()
        if not upcoming_waypoints:
            return None
        rosys.notify(f'Upcoming waypoints: {upcoming_waypoints}')
        # If only one waypoint is left, use the relative heading from robot_pose
        if len(upcoming_waypoints) == 1:
            target_point = upcoming_waypoints[0]
            target_heading = self.robot_locator.pose.direction(target_point)
            return rosys.geometry.Pose(x=target_point.x, y=target_point.y, yaw=target_heading)

        # If multiple waypoints, calculate target pose based on the path
        # Use the first upcoming waypoint as target position
        target_point = upcoming_waypoints[0]

        # Calculate heading based on the direction to the next waypoint
        if len(upcoming_waypoints) > 1:
            # Use direction from current target to next waypoint for heading
            next_point = upcoming_waypoints[1]
            target_heading = target_point.direction(next_point)
        else:
            # Fallback to direction from robot to target point
            target_heading = self.robot_locator.pose.direction(target_point)

        return rosys.geometry.Pose(x=target_point.x, y=target_point.y, yaw=target_heading)

    def update_target(self) -> None:
        self.origin = self.robot_locator.pose.point
        if self.current_row is not None:
            self.target = self.get_target().point

    @track
    async def _drive(self) -> None:
        assert self.field is not None
        if self._state == State.FOLLOW_ROW:
            self._state = await self._run_follow_row()
            return
        with self.implement.blocked():
            if self._state == State.APPROACH_START_ROW:
                self._state = await self._run_approach_start_row()
            elif self._state == State.CHANGE_ROW:
                self._state = await self._run_change_row()
            elif self._state == State.ROW_COMPLETED:
                self._state = await self._run_row_completed()
            elif self._state == State.WAITING_FOR_CONFIRMATION:
                self._state = await self._run_waiting_for_confirmation()

    @track
    async def _run_approach_start_row(self) -> State:
        self.allowed_to_turn = False
        # self.set_waypoints()
        # if self.start_point is None or self.end_point is None:
        #     return State.ERROR
        # is_in_row = self._is_in_working_area(self.start_point, self.end_point, position_error_margin=0.0)
        # if not self._is_start_allowed(self.start_point, self.end_point, is_in_row):
        #     return State.ERROR
        # if isinstance(self.detector, rosys.vision.DetectorSimulation) and not rosys.is_test:
        #     self.create_simulation()
        # else:
        #     self.plant_provider.clear()
        target = self.get_target()
        await self.turn_to_yaw(target.yaw)
        await self.driver.drive_to(target.point, backward=False)
        driving_yaw = self.robot_locator.pose.direction(target.point)
        await self.turn_to_yaw(driving_yaw)
        self._set_cultivated_crop()
        return State.FOLLOW_ROW

    @track
    async def _run_change_row(self) -> State:
        self.set_waypoints()
        assert self.start_point is not None
        target_yaw = self.robot_locator.pose.direction(self.start_point)
        await self.turn_to_yaw(target_yaw)
        await self.drive_towards_target(rosys.geometry.Pose(x=self.start_point.x, y=self.start_point.y, yaw=target_yaw), target_heading=target_yaw)
        assert self.end_point is not None
        row_yaw = self.start_point.direction(self.end_point)
        await self.turn_to_yaw(row_yaw)
        if isinstance(self.detector, rosys.vision.DetectorSimulation) and not rosys.is_test:
            self.create_simulation()
        else:
            self.plant_provider.clear()
        self._set_cultivated_crop()
        self.allowed_to_turn = False
        return State.FOLLOW_ROW

    @track
    async def _run_follow_row(self) -> State:
        assert self.end_point is not None
        assert self.start_point is not None
        upcoming = self.get_upcoming_waypoints()
        if upcoming is None:
            self.log.warning('No upcoming waypoint found')
            return State.ERROR
        end_pose = rosys.geometry.Pose(x=upcoming.x, y=upcoming.y, yaw=upcoming.direction(self.end_point), time=0)
        distance_from_end = end_pose.relative_point(self.robot_locator.pose.point).x
        if distance_from_end > 0:
            await self.driver.wheels.stop()
            await self.implement.deactivate()
            return State.ROW_COMPLETED
        if self.is_in_swarm and not self.allowed_to_turn and -self.wait_distance < distance_from_end < 0:
            await self.driver.wheels.stop()
            return State.WAITING_FOR_CONFIRMATION
        if not self.implement.is_active:
            gc.collect()
            await self.implement.activate()
        self.update_target()
        await self.drive_towards_target(end_pose)
        return State.FOLLOW_ROW

    @track
    async def _run_waiting_for_confirmation(self) -> State:
        await self.driver.wheels.stop()
        await rosys.sleep(0.1)
        if self.allowed_to_turn:
            return State.FOLLOW_ROW
        return State.WAITING_FOR_CONFIRMATION

    @track
    async def _run_row_completed(self) -> State:
        await self.driver.wheels.stop()
        assert self.field
        next_state: State = State.ROW_COMPLETED
        if not self._loop and self.current_row == self.rows_to_work_on[-1]:
            return State.FIELD_COMPLETED
        self.row_index += 1
        next_state = State.CHANGE_ROW

        # TODO: rework later, when starting at any row is possible
        if self.row_index >= len(self.rows_to_work_on):
            if self._loop:
                self.row_index = 0
            else:
                next_state = State.FIELD_COMPLETED
        return next_state

    def _set_cultivated_crop(self) -> None:
        if not isinstance(self.implement, WeedingImplement):
            self.log.warning('Implement is not a weeding implement')
            return
        if self.current_row is None:
            self.log.warning('No current row')
            return
        if self.implement.cultivated_crop == self.current_row.crop:
            return
        self.implement.cultivated_crop = self.current_row.crop
        self.implement.request_backup()

    def _is_in_working_area(self, start_point: Point, end_point: Point, *, position_error_margin: float = 0.05) -> bool:
        # TODO: check if in working rectangle, current just checks if between start and stop
        relative_start = self.robot_locator.pose.relative_point(start_point)
        relative_end = self.robot_locator.pose.relative_point(end_point)
        robot_in_working_area = relative_start.x * relative_end.x <= 0.0
        if abs(relative_start.x) < position_error_margin or abs(relative_end.x) < position_error_margin:
            return False
        return robot_in_working_area

    def _is_start_allowed(self, start_point: Point, end_point: Point, robot_in_working_area: bool) -> bool:
        if not robot_in_working_area:
            self.log.warning('Robot is not in working area')
            return True
        if self.current_row is None:
            self.log.warning('No current row')
            return False
        foot_point = self.current_row.line_segment().line.foot_point(self.robot_locator.pose.point)
        distance_to_row = foot_point.distance(self.robot_locator.pose.point)
        if distance_to_row > self.MAX_DISTANCE_DEVIATION:
            rosys.notify('Robot is between two rows', 'negative')
            return False
        abs_angle_to_start = abs(self.robot_locator.pose.relative_direction(start_point))
        abs_angle_to_end = abs(self.robot_locator.pose.relative_direction(end_point))
        if abs_angle_to_start > self.MAX_ANGLE_DEVIATION and abs_angle_to_end > self.MAX_ANGLE_DEVIATION:
            rosys.notify('Robot heading deviates too much from row direction', 'negative')
            return False
        return True

    def backup_to_dict(self) -> dict[str, Any]:
        return super().backup_to_dict() | {
            'field_id': self.field.source.id if self.field else None,
            'loop': self._loop,
            'wait_distance': self.wait_distance,
            'force_first_row_start': self.force_first_row_start,
            'is_in_swarm': self.is_in_swarm,
        }

    def restore_from_dict(self, data: dict[str, Any]) -> None:
        super().restore_from_dict(data)
        field_id = data.get('field_id', self.field_provider.fields[0].source.id if self.field_provider.fields else None)
        self.field = self.field_provider.get_field(field_id)
        self._loop = data.get('loop', False)
        self.force_first_row_start = data.get('force_first_row_start', self.force_first_row_start)
        self.wait_distance = data.get('wait_distance', self.wait_distance)
        self.is_in_swarm = data.get('is_in_swarm', self.is_in_swarm)

    def settings_ui(self) -> None:
        with ui.row():
            super().settings_ui()
            ui.button(icon='add_box', text='Waypoint', on_click=self.add_waypoint)
            with ui.row():
                ui.label('working on:')
                ui.label().classes('w-12') \
                    .bind_text_from(self, 'row_index', lambda i: self.rows_to_work_on[i].name if i < len(self.rows_to_work_on) else '-')
                ui.label().classes('w-12') \
                    .bind_text_from(self, '_state', lambda state: state.name)

    def developer_ui(self) -> None:
        # super().developer_ui()
        ui.label('Field Navigation').classes('text-center text-bold')
        ui.label('').bind_text_from(self, '_state', lambda state: f'State: {state.name}')
        ui.label('').bind_text_from(self, 'row_index', lambda row_index: f'Row Index: {row_index}')
        ui.checkbox('Loop', on_change=self.request_backup).bind_value(self, '_loop')
        ui.checkbox('Force first row start', on_change=self.request_backup).bind_value(self, 'force_first_row_start')
        ui.checkbox('Is in swarm', on_change=self.request_backup).bind_value(self, 'is_in_swarm')
        ui.checkbox('Allowed to turn').bind_value(self, 'allowed_to_turn')
        ui.number('Wait distance', step=0.1, min=0.0, max=10.0, format='%.1f', suffix='m', on_change=self.request_backup) \
            .bind_value(self, 'wait_distance').classes('w-20')

    def _set_field_id(self) -> None:
        self.field_id = self.field_provider.selected_field.source.id if self.field_provider.selected_field else None

    def add_waypoint(self) -> bool:
        if not self.robot_locator.pose:
            rosys.notify('missing robot pose', 'negative')
            return False
        if self._state != State.FOLLOW_ROW:
            rosys.notify('you must be following a row', 'negative')
            return False
        target_row = self.rows_to_work_on[self.row_index]
        robot_point = self.robot_locator.pose.point

        row_start = target_row.points[0].to_local()
        if robot_point.distance(row_start) < 1.0:
            waypoint_index = 0
        else:
            existing_waypoints = [sp for sp in self.field.source.row_support_points
                                  if sp.row_index == self.row_index]
            closest_waypoint = None
            for waypoint in existing_waypoints:
                if robot_point.distance(waypoint.to_local()) < 1.0:
                    closest_waypoint = waypoint
                    break
            if closest_waypoint:
                waypoint_index = closest_waypoint.waypoint_index
            else:
                used_indices = {wp.waypoint_index for wp in existing_waypoints}
                waypoint_index = 1
                while waypoint_index in used_indices:
                    waypoint_index += 1

        geo_point = rosys.geometry.GeoPoint.from_point(robot_point)
        row_support_point = RowSupportPoint.from_geopoint(
            geo_point,
            row_index=self.row_index,
            waypoint_index=waypoint_index
        )
        self.field_provider.add_row_support_point(self.field.source.id, row_support_point)
        self.log.info('Added waypoint %d for row %d at current position', waypoint_index, self.row_index)
        return True

    def create_simulation(self, crop_distance: float = 0.3) -> None:
        assert isinstance(self.detector, rosys.vision.DetectorSimulation)
        self.detector.simulated_objects.clear()
        self.plant_provider.clear()
        if self.field is None:
            return
        if self.start_point is not None and self.end_point is not None:
            length = self.start_point.distance(self.end_point)
            crop_count = length / crop_distance
            assert self.current_row is not None
            crop = self.current_row.crop or 'maize'
            for i in range(int(crop_count)):
                p = self.start_point.interpolate(self.end_point, (crop_distance * (i+1)) / length)
                if i == 10:
                    p.y += 0.20
                else:
                    p.y += randint(-5, 5) * 0.01
                p3d = rosys.geometry.Point3d(x=p.x, y=p.y, z=0)
                plant = rosys.vision.SimulatedObject(category_name=crop, position=p3d)
                self.detector.simulated_objects.append(plant)

                for _ in range(1, 7):
                    p = self.start_point.polar(crop_distance * (i+1) + randint(-5, 5) * 0.01, self.start_point.direction(self.end_point)) \
                        .polar(randint(-15, 15)*0.01, self.robot_locator.pose.yaw + np.pi/2)
                    self.detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='weed',
                                                                                        position=rosys.geometry.Point3d(x=p.x, y=p.y, z=0)))
