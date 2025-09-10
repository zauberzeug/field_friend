from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Self

import numpy as np
import rosys
from nicegui import ui
from rosys import helpers
from rosys.analysis import track
from rosys.geometry import Pose
from rosys.hardware import BmsHardware, BmsSimulation
from shapely.geometry import Point as ShapelyPoint
from shapely.geometry import Polygon as ShapelyPolygon

from ..field import Field, Row
from ..implements import Implement, WeedingImplement
from .waypoint_navigation import DriveSegment, WaypointNavigation

if TYPE_CHECKING:
    from ...system import System


class FieldNavigation(WaypointNavigation):
    MAX_START_DISTANCE = 1.0
    MAX_DISTANCE_DEVIATION = 0.1
    MAX_ANGLE_DEVIATION = np.deg2rad(15.0)
    DOCKING_SPEED = 0.1
    BATTERY_CHARGE_PERCENTAGE = 30.0
    BATTERY_WORKING_PERCENTAGE = 85.0
    START_ROW_INDEX = 0
    RETURN_TO_START = True
    CHARGE_AUTOMATICALLY = False

    def __init__(self, system: System, implement: Implement) -> None:
        super().__init__(system, implement)
        self.name = 'Field Navigation'
        self.gnss = system.gnss
        self.automator = system.automator
        self.automation_watcher = system.automation_watcher
        self.field_provider = system.field_provider

        self.battery_charge_percentage = self.BATTERY_CHARGE_PERCENTAGE
        self.battery_working_percentage = self.BATTERY_WORKING_PERCENTAGE
        self.start_row_index = self.START_ROW_INDEX
        self.return_to_start = self.RETURN_TO_START
        self.charge_automatically = self.CHARGE_AUTOMATICALLY
        self.force_charge = False

    @property
    def field(self) -> Field | None:
        return self.field_provider.selected_field

    @property
    def current_row(self) -> Row | None:
        return self.current_segment.row if isinstance(self.current_segment, RowSegment) else None

    def _handle_segment_started(self, segment: DriveSegment) -> None:
        super()._handle_segment_started(segment)
        if isinstance(segment, RowSegment) and isinstance(self.implement, WeedingImplement):
            self.log.debug(f'Setting crop to {segment.row.crop}')
            self.implement.cultivated_crop = segment.row.crop

    @track
    async def prepare(self) -> bool:
        await super().prepare()
        if not self._is_allowed_to_start():
            return False
        self.system.automation_watcher.gnss_watch_active = True
        if self.current_segment is not None:
            self._handle_segment_started(self.current_segment)
        return True

    def generate_path(self) -> list[DriveSegment | RowSegment]:
        field_id: str | None = self.field_provider.selected_field.id if self.field_provider.selected_field else None
        field = self.field_provider.get_field(field_id)
        if field is None:
            rosys.notify('No field selected', 'negative')
            return []
        rows_to_work_on = self.field_provider.get_rows_to_work_on()
        if not rows_to_work_on:
            rosys.notify('No rows to work on', 'negative')
            return []
        path_segments: list[DriveSegment | RowSegment] = []
        current_pose = self.system.robot_locator.pose
        closest_row_index = self._find_closest_row_index(rows_to_work_on)
        distance_to_closest_row = self._distance_to_row(rows_to_work_on[closest_row_index])
        start_row_index = closest_row_index if distance_to_closest_row <= self.MAX_START_DISTANCE else \
            int(self.start_row_index)
        if start_row_index >= len(rows_to_work_on):
            rosys.notify('Start row index is out of range', 'negative')
            return []
        row_reversed = self._is_row_reversed(rows_to_work_on[start_row_index])

        turn_start = current_pose
        for i, row in enumerate(rows_to_work_on):
            if i < start_row_index:
                continue
            row_segment = RowSegment.from_row(row, reverse=row_reversed)
            if path_segments:
                path_segments.extend(self._generate_three_point_turn(turn_start, row_segment.start))
            path_segments.append(row_segment)
            turn_start = row_segment.end
            row_reversed = not row_reversed

        assert isinstance(path_segments[0], RowSegment)
        t = path_segments[0].spline.closest_point(current_pose.x, current_pose.y)
        distance = current_pose.distance(path_segments[0].spline.pose(t))
        if distance > self.MAX_DISTANCE_DEVIATION:
            path_segments = self._generate_row_approach_path(path_segments[0].row) + path_segments

        if self.return_to_start:
            assert self.field is not None
            first_row = self.field.rows[0]
            if row_reversed:
                # NOTE: last row should not be reversed, but it is flipped at the end of the last loop
                assert self.field_provider.selected_field is not None
                assert isinstance(path_segments[-1], RowSegment)
                end_pose = path_segments[-1].end
                row_start = first_row.points[0].to_local()
                row_end = first_row.points[-1].to_local()
                row_end_pose = Pose(x=row_end.x, y=row_end.y, yaw=row_end.direction(row_start))
                turn_segments = self._generate_three_point_turn(end_pose, row_end_pose)
                drive_segment = RowSegment.from_row(first_row, reverse=True)
                drive_segment.use_implement = False
                path_segments = [*path_segments, *turn_segments, drive_segment]

            # NOTE: align with first row
            first_row_segment = RowSegment.from_row(first_row)
            turn_segments = self._generate_three_point_turn(path_segments[-1].end, first_row_segment.start)
            path_segments = [*path_segments, *turn_segments]
        return path_segments

    @track
    async def finish(self) -> None:
        await super().finish()
        self.system.automation_watcher.gnss_watch_active = False

    async def _run(self) -> None:
        assert self.field is not None
        if self._should_charge():
            no_more_rows = sum(1 for segment in self._upcoming_path if isinstance(segment, RowSegment)) == 0
            await self._run_charging(stop_after_docking=no_more_rows)
        if self.field.charging_station is not None and self.has_waypoints and self.system.field_friend.bms.state.is_charging:
            await self.undock()
            while not isinstance(self.current_segment, RowSegment):
                self._upcoming_path.pop(0)
            if isinstance(self.current_segment, RowSegment):
                self._upcoming_path = self._generate_row_approach_path(self.current_segment.row) + self._upcoming_path
                self.PATH_GENERATED.emit(self._upcoming_path)
        await super()._run()

    def _should_charge(self) -> bool:
        assert self.field is not None
        if not self.charge_automatically:
            return False
        if self.field.charging_station is None:
            return False
        if self.current_row:
            self.log.debug('Not charging: Not allowed to charge on row')
            return False
        closest_row_index = self._find_closest_row_index(self.field_provider.get_rows_to_work_on())
        closest_row = self.field_provider.get_rows_to_work_on()[closest_row_index]
        closest_row_start = closest_row.points[0].to_local()
        closest_row_end = closest_row.points[-1].to_local()
        current_pose = self.system.robot_locator.pose
        if closest_row_start.distance(current_pose.point) > 0.1:
            return False
        if sum(1 for segment in self._upcoming_path if isinstance(segment, RowSegment)) == 0:
            return True
        direction_to_start = current_pose.relative_direction(closest_row_start)
        direction_to_end = current_pose.relative_direction(closest_row_end)
        if abs(direction_to_start) <= self.MAX_ANGLE_DEVIATION and abs(direction_to_end) <= self.MAX_ANGLE_DEVIATION:
            self.log.debug('Not charging: Robot in front of new row')
            return False
        if self.force_charge:
            return True
        return self.system.field_friend.bms.is_below_percent(self.BATTERY_CHARGE_PERCENTAGE)

    @track
    async def _run_charging(self, *, stop_after_docking: bool = False) -> None:
        assert self.field is not None
        assert self.field.charging_station is not None
        assert self.field.charge_approach_pose is not None
        while self.current_segment is not None and not isinstance(self.current_segment, RowSegment):
            self._upcoming_path.pop(0)  # NOTE: pop unnecessary turn segments
        approach_pose = self.field.charging_station.approach_pose.to_local()
        approach_segment = DriveSegment.from_poses(self.system.robot_locator.pose, approach_pose)
        self._upcoming_path.insert(0, approach_segment)
        self.PATH_GENERATED.emit(self._upcoming_path)
        await self._drive_along_segment()
        await self.dock()
        if stop_after_docking:
            return
        while self.system.field_friend.bms.is_below_percent(self.BATTERY_WORKING_PERCENTAGE) or self.force_charge:
            await rosys.sleep(1)

    def _find_closest_row_index(self, rows: list[Row]) -> int:
        """Find the index of the closest row to the current position"""
        robot_pose = self.system.robot_locator.pose
        shortest_row_distances = [min(robot_pose.distance(row.points[0].to_local()),
                                      robot_pose.distance(row.points[-1].to_local())) for row in rows]
        return int(np.argmin(shortest_row_distances))

    def _is_row_reversed(self, row: Row) -> bool:
        current_pose = self.system.robot_locator.pose
        row_start = row.points[0].to_local()
        row_end = row.points[-1].to_local()
        relative_start = current_pose.relative_point(row_start)
        relative_end = current_pose.relative_point(row_end)
        robot_on_headland = relative_start.x * relative_end.x > 0.0
        if robot_on_headland:
            distance_to_start = current_pose.distance(row_start)
            distance_to_end = current_pose.distance(row_end)
            return distance_to_start > distance_to_end
        return relative_start.x > 0

    def _distance_to_row(self, row: Row) -> float:
        current_pose = self.system.robot_locator.pose
        row_segment = RowSegment.from_row(row)
        current_t = row_segment.spline.closest_point(current_pose.x, current_pose.y)
        current_pose_on_row = row_segment.spline.pose(current_t)
        return current_pose.distance(current_pose_on_row)

    def _is_allowed_to_start(self) -> bool:
        first_row_segment = next((segment for segment in self._upcoming_path if isinstance(segment, RowSegment)), None)
        if first_row_segment is None:
            return False
        if not self.system.automation_watcher.is_gnss_ready():
            rosys.notify('GNSS quality is not sufficient', 'negative')
            return False
        current_pose = self.system.robot_locator.pose
        assert self.field_provider.selected_field is not None
        field_polygon = ShapelyPolygon([point.to_local().tuple for point in self.field_provider.selected_field.outline])
        if not field_polygon.contains(ShapelyPoint(current_pose.x, current_pose.y)):
            rosys.notify('Robot is outside of field boundaries', 'negative')
            return False
        relative_start = current_pose.relative_point(first_row_segment.start.point)
        relative_end = current_pose.relative_point(first_row_segment.end.point)
        robot_on_headland = relative_start.x * relative_end.x > 0.0
        if not robot_on_headland:
            t = first_row_segment.spline.closest_point(current_pose.x, current_pose.y)
            spline_pose = first_row_segment.spline.pose(t)
            if current_pose.distance(spline_pose) > self.MAX_DISTANCE_DEVIATION:
                rosys.notify('Distance to row is too large', 'negative')
                return False
            if abs(helpers.angle(current_pose.yaw, spline_pose.yaw)) > self.MAX_ANGLE_DEVIATION:
                rosys.notify('Robot is not aligned with the row', 'negative')
                return False
        return True

    def _generate_row_approach_path(self, row: Row, *, safety_padding: float = 0.1) -> list[DriveSegment]:
        assert self.field is not None
        local_row_start = row.points[0].to_local()
        local_row_end = row.points[-1].to_local()
        row_start_pose = Pose(x=local_row_start.x, y=local_row_start.y, yaw=local_row_start.direction(local_row_end))
        if self.charge_automatically and self.system.field_friend.bms.state.is_charging:
            assert self.field.charging_station is not None
            current_pose = self.field.charging_station.approach_pose.to_local()
        else:
            current_pose = self.system.robot_locator.pose
        if current_pose.distance(row_start_pose) < safety_padding:
            return [DriveSegment.from_poses(current_pose, row_start_pose)]
        end_pose = row_start_pose.transform_pose(Pose(x=-safety_padding, y=0.0, yaw=0.0))
        row_approach_segment = DriveSegment.from_poses(current_pose, end_pose)
        return [
            row_approach_segment,
            DriveSegment.from_poses(row_approach_segment.end, row_start_pose),
        ]

    def _generate_three_point_turn(self, end_pose_current_row: Pose, start_pose_next_row: Pose, radius: float = 1.5) -> list[DriveSegment]:
        direction_to_start = end_pose_current_row.relative_direction(start_pose_next_row)
        if end_pose_current_row.distance(start_pose_next_row) < 0.01:
            direction_to_start = np.deg2rad(90)
        first_turn_pose = end_pose_current_row.transform_pose(
            Pose(x=radius, y=radius * np.sign(direction_to_start), yaw=direction_to_start))
        back_up_pose = start_pose_next_row.transform_pose(
            Pose(x=-radius, y=radius * np.sign(direction_to_start), yaw=-direction_to_start))
        backward = first_turn_pose.relative_pose(back_up_pose).x < 0
        return [
            DriveSegment.from_poses(end_pose_current_row, first_turn_pose, stop_at_end=backward),
            DriveSegment.from_poses(first_turn_pose, back_up_pose, backward=backward, stop_at_end=backward),
            DriveSegment.from_poses(back_up_pose, start_pose_next_row),
        ]

    @track
    async def approach_dock(self):
        assert self.field is not None
        assert self.field.charging_station is not None
        approach_pose = self.field.charging_station.approach_pose.to_local()
        forward_segment = DriveSegment.from_poses(self.system.robot_locator.pose, approach_pose)
        forward_length = forward_segment.spline.estimated_length()
        backward_segment = DriveSegment.from_poses(self.system.robot_locator.pose, approach_pose, backward=True)
        backward_length = backward_segment.spline.estimated_length()
        approach_segment = forward_segment if forward_length <= backward_length else backward_segment
        self._upcoming_path.insert(0, approach_segment)
        self.PATH_GENERATED.emit(self._upcoming_path)
        await self._drive_along_segment(linear_speed_limit=self.DOCKING_SPEED)

    @track
    async def dock(self):
        async def wait_for_charging():
            while not self.system.field_friend.bms.state.is_charging:
                await rosys.sleep(0.1)
            self.log.debug('Charging station detected, stopping')

        async def gnss_move(y_offset: float = 0.0):
            assert self.system.gnss is not None
            assert self.system.gnss.last_measurement is not None
            if not self.system.automation_watcher.is_gnss_ready():
                self.log.error('No RTK fix, aborting')
                return
            assert self.field is not None
            assert self.field.charging_station is not None
            self.log.debug('Moving to docked pose: %s with y_offset: %s',
                           self.field.charging_station.dock_pose, y_offset)
            local_docked_pose = self.field.charging_station.dock_pose.to_local() \
                .transform_pose(Pose(x=0.0, y=y_offset, yaw=0.0))
            docking_segment = DriveSegment.from_poses(self.system.robot_locator.pose, local_docked_pose, backward=True)
            self._upcoming_path.insert(0, docking_segment)
            self.PATH_GENERATED.emit(self._upcoming_path)
            await self._drive_along_segment(linear_speed_limit=self.DOCKING_SPEED)
            if isinstance(self.system.field_friend.bms, BmsSimulation):
                self.system.field_friend.bms.voltage_per_second = 0.03

        rosys.notify('Docking to charging station')
        if isinstance(self.system.field_friend.bms, BmsHardware):
            old_interval = self.system.field_friend.bms.UPDATE_INTERVAL
            self.system.field_friend.bms.UPDATE_INTERVAL = 0.1
        for y_offset in (0.0, 0.0, 0.005, -0.005, 0.01, -0.01, 0.015, -0.015, 0.02, -0.02):
            await rosys.automation.parallelize(
                gnss_move(y_offset),
                wait_for_charging(),
                return_when_first_completed=True,
            )
            await rosys.automation.parallelize(
                rosys.sleep(20),
                wait_for_charging(),
                return_when_first_completed=True,
            )
            if self.system.field_friend.bms.state.is_charging:
                rosys.notify('Docking successful', 'positive')
                break
            await self.undock()
        else:
            rosys.notify('Failed to dock', 'negative')
        if isinstance(self.system.field_friend.bms, BmsHardware):
            self.system.field_friend.bms.UPDATE_INTERVAL = old_interval
        await self.system.field_friend.wheels.stop()

    @track
    async def undock(self):
        assert self.field is not None
        assert self.field.charging_station is not None
        if self.field.charging_station.approach_pose is None:
            rosys.notify('Record the docked position first', 'negative')
            return
        rosys.notify('Detaching from charging station')
        undock_pose = self.field.charging_station.approach_pose.to_local()
        if isinstance(self.system.field_friend.bms, BmsSimulation):
            self.system.field_friend.bms.voltage_per_second = -0.01
        undock_segment = DriveSegment.from_poses(self.system.robot_locator.pose, undock_pose)
        self._upcoming_path.insert(0, undock_segment)
        self.PATH_GENERATED.emit(self._upcoming_path)
        await self._drive_along_segment(linear_speed_limit=self.DOCKING_SPEED)

    def backup_to_dict(self) -> dict[str, Any]:
        return super().backup_to_dict() | {
            'return_to_start': self.return_to_start,
            'charge_automatically': self.charge_automatically,
            'battery_charge_percentage': self.battery_charge_percentage,
            'battery_working_percentage': self.battery_working_percentage,
        }

    def restore_from_dict(self, data: dict[str, Any]) -> None:
        super().restore_from_dict(data)
        self.return_to_start = data.get('return_to_start', self.RETURN_TO_START)
        self.charge_automatically = data.get('charge_automatically', self.CHARGE_AUTOMATICALLY)
        self.battery_charge_percentage = data.get('battery_charge_percentage', self.BATTERY_CHARGE_PERCENTAGE)
        self.battery_working_percentage = data.get('battery_working_percentage', self.BATTERY_WORKING_PERCENTAGE)

    def settings_ui(self) -> None:
        super().settings_ui()
        ui.number('Start row', min=0, step=1, value=self.start_row_index, on_change=self.request_backup) \
            .props('dense outlined') \
            .classes('w-20') \
            .bind_value(self, 'start_row_index', forward=int) \
            .tooltip('The row index from which the robot should start working. If the robot is already on a row, it will start from it instead')
        ui.checkbox('Return to start', on_change=self.request_backup) \
            .bind_value(self, 'return_to_start') \
            .bind_value_from(self, 'charge_automatically', lambda value: True if value else self.return_to_start) \
            .tooltip('The robot will return to the start row')
        ui.checkbox('Charge automatically', on_change=self.request_backup) \
            .bind_value(self, 'charge_automatically',
                        forward=lambda v: v and self.field is not None and self.field.charging_station is not None,
                        backward=lambda v: v and self.field is not None and self.field.charging_station is not None) \
            .bind_visibility_from(self, 'field', lambda field: field is not None and field.charging_station is not None) \
            .tooltip('Let the robot charge automatically when a charging station is provided')

    def developer_ui(self):
        ui.label('Field Navigation').classes('text-center text-bold')
        ui.button('Approach', on_click=lambda: self.system.automator.start(self.approach_dock()))
        ui.button('Dock', on_click=lambda: self.system.automator.start(self.dock()))
        ui.button('Undock', on_click=lambda: self.system.automator.start(self.undock()))
        ui.number('Charge percentage', min=0, max=100, step=1, value=self.battery_charge_percentage, on_change=self.request_backup) \
            .props('dense outlined') \
            .classes('w-32') \
            .bind_value(self, 'battery_charge_percentage') \
            .tooltip('Battery charge percentage at which the robot should charge automatically')
        ui.number('Working percentage', min=0, max=100, step=1, value=self.battery_working_percentage, on_change=self.request_backup) \
            .props('dense outlined') \
            .classes('w-32') \
            .bind_value(self, 'battery_working_percentage') \
            .tooltip('Battery charge percentage at which the robot is allowed to stop charging and continue working')
        ui.checkbox('Force charge', on_change=self.request_backup) \
            .bind_value(self, 'force_charge') \
            .tooltip('Force the robot to charge even if it is not below the working percentage')


@dataclass(slots=True, kw_only=True)
class RowSegment(DriveSegment):
    row: Row

    @classmethod
    def from_row(cls, row: Row, *, reverse: bool = False) -> Self:
        start_point = row.points[0].to_local()
        end_point = row.points[-1].to_local()
        if reverse:
            start_point, end_point = end_point, start_point
        start_pose = Pose(x=start_point.x, y=start_point.y, yaw=start_point.direction(end_point))
        end_pose = Pose(x=end_point.x, y=end_point.y, yaw=start_point.direction(end_point))
        segment = DriveSegment.from_poses(start_pose, end_pose, use_implement=True)
        return cls(row=row, spline=segment.spline, use_implement=segment.use_implement, backward=segment.backward, stop_at_end=segment.stop_at_end)
