from typing import Any, Optional

import numpy as np
import rosys
from rosys.driving import PathSegment
from rosys.geometry import Point, Pose, Spline

from ...navigation import Gnss
from ..field import Field, Row
from ..sequence import find_sequence
from ..tool.tool import Tool
from .navigation import Navigation


class FieldNavigation(Navigation):

    def __init__(self,
                 driver: rosys.driving.Driver,
                 odometer: rosys.driving.Odometer,
                 tool: Tool,
                 shape: rosys.geometry.Prism,
                 gnss:  Gnss,
                 bms: rosys.hardware.Bms) -> None:
        super().__init__(driver, odometer, tool)

        self.PATH_PLANNED = rosys.event.Event()
        '''Event that is emitted when the path is planed. The event contains the path as a list of PathSegments.'''

        self.gnss = gnss
        self.bms = bms
        self.path_planner = rosys.pathplanning.PathPlanner(shape)
        self.continue_canceled_weeding: bool = False

        self.field: Optional[Field] = None
        self.start_row_id: Optional[str] = None
        self.end_row_id: Optional[str] = None
        self.minimum_turning_radius: float = 1.8
        self.turn_offset: float = 1.0
        self.drive_backwards_to_start: bool = True
        self.drive_to_start: bool = True

        self.weeding_plan: list[list[PathSegment]] = []
        self.sorted_weeding_rows: list = []
        self.turn_paths: list[list[PathSegment]] = []
        self.current_row: Optional[Row] = None
        self.current_segment: Optional[PathSegment] = None
        self.row_segment_completed: bool = False

        # driver settings
        self.linear_speed_on_row: float = 0.04
        self.angular_speed_on_row: float = 0.3
        self.linear_speed_between_rows: float = 0.3
        self.angular_speed_between_rows: float = 0.8

    async def start(self) -> None:
        if not await self.tool.prepare():
            self.log.error('Tool-Preparation failed')
            return
        if not await self._prepare():
            self.log.error('Navigation-Preparation failed')
            return
        for i, path in enumerate(self.weeding_plan):
            if self.continue_canceled_weeding and self.current_row != self.sorted_weeding_rows[i]:
                continue
            self.driver.parameters.can_drive_backwards = False
            self.driver.parameters.linear_speed_limit = self.linear_speed_on_row
            self.driver.parameters.angular_speed_limit = self.angular_speed_on_row
            self.current_row = self.sorted_weeding_rows[i]
            await self.tool.activate()
            for j, segment in enumerate(path):
                if self.continue_canceled_weeding and self.current_segment != segment:
                    continue
                else:
                    self.continue_canceled_weeding = False
                self.current_segment = segment
                # self.invalidate()
                self.log.info(f'Driving row {i + 1}/{len(self.weeding_plan)} and segment {j + 1}/{len(path)}...')
                self.row_segment_completed = False
                while not self.row_segment_completed:
                    self.log.info('while not row completed...')
                    await rosys.automation.parallelize(
                        self.tool.observe(),
                        self._drive_segment(),
                        return_when_first_completed=True
                    )
                    await self.tool.on_focus()
                    if self.odometer.prediction.relative_point(self.current_segment.spline.end).x < 0.01:
                        self.row_segment_completed = True
                    await rosys.sleep(0.2)
                    if self.drive_backwards_to_start and self.bms.is_below_percent(15.0):
                        self.log.info('Low battery, driving backwards to start...')
                        rosys.notify('Low battery, driving backwards to start', 'warning')
                        await self.driver.drive_to(Point(x=self.weeding_plan[0][0].spline.start.x, y=self.weeding_plan[0][0].spline.start.y), backward=True)
                        return

            await self.tool.deactivate()
            if i < len(self.weeding_plan) - 1:
                self.driver.parameters.can_drive_backwards = True
                self.driver.parameters.linear_speed_limit = self.linear_speed_between_rows
                self.driver.parameters.angular_speed_limit = self.angular_speed_between_rows
                self.log.info('Driving to next row...')
                turn_path = self.turn_paths[i]
                await self.driver.drive_path(turn_path)
                await rosys.sleep(1)

        # TODO reactivate automation watch
        # self.system.automation_watcher.stop_field_watch()
        # self.system.automation_watcher.gnss_watch_active = False
        self.sorted_weeding_rows = []
        self.weeding_plan = []
        self.turn_paths = []
        self.current_row = None
        self.current_segment = None

    async def _prepare(self) -> bool:
        if not self.continue_canceled_weeding:
            if not await self._field_planning():
                rosys.notify('Field planning failed', 'negative')
                return False
        if self.continue_canceled_weeding:
            start_pose = self.odometer.prediction
            assert self.current_segment is not None
            end_pose = Pose(x=self.current_segment.spline.start.x, y=self.current_segment.spline.start.y,
                            yaw=self.current_segment.spline.start.direction(self.current_segment.spline.end))
            start_spline = Spline.from_poses(start_pose, end_pose)
            await self.driver.drive_spline(start_spline)
        elif self.drive_to_start:
            await self._drive_to_start()
        # TODO: reactivate automation watcher
        # self.system.automation_watcher.start_field_watch(self.field.outline)
        # self.system.automation_watcher.gnss_watch_active = True
        return True

    async def _field_planning(self) -> bool:
        if self.gnss.device is None:
            self.log.error('GNSS is not available')
            return False
        if self.field is None:
            self.log.error('Field is not available')
            rosys.notify('No field selected', 'negative')
            return False
        if not self.field.reference:
            self.log.error('Field reference is not available')
            return False
        self.gnss.reference = self.field.reference
        self.weeding_plan = self._make_plan()
        if not self.weeding_plan:
            self.log.error('No plan available')
            return False
        self.turn_paths = await self._generate_turn_paths()
        if not self.turn_paths:
            self.log.error('No turn paths available')
        self.PATH_PLANNED.emit()
        return True

    def _make_plan(self) -> list[list[rosys.driving.PathSegment]]:
        self.log.info('Making plan...')
        if self.field is None:
            self.log.warning('No field available')
            return []
        if not self.field.rows:
            self.log.warning('No rows available')
            return []
        start_row = next((row for row in self.field.rows if row.id == self.start_row_id), None)
        end_row = next((row for row in self.field.rows if row.id == self.end_row_id), None)
        if start_row is None:
            start_row = self.field.rows[0]
        if end_row is None:
            end_row = self.field.rows[-1]
        self.start_row_id = start_row.id
        self.end_row_id = end_row.id
        reference = self.field.reference
        assert reference is not None
        rows_to_weed = self.field.rows[self.field.rows.index(start_row):self.field.rows.index(end_row) + 1]
        rows = [row for row in rows_to_weed if len(row.cartesian(reference)) > 1]
        robot_position = self.odometer.prediction.point
        distance_to_first_row = min([point.distance(robot_position) for point in rows[0].cartesian(reference)])
        distance_to_last_row = min([point.distance(robot_position) for point in rows[-1].cartesian(reference)])
        if distance_to_first_row > distance_to_last_row:
            rows = list(reversed(rows))
        minimum_row_distance = 1  # 1 = no row needs to be skipped when turning
        if len(rows) > 1:
            rows_distance = rows[0].cartesian(reference)[0].distance(rows[1].cartesian(reference)[0])
            self.log.info(f'Rows distance: {rows_distance}')
            self.log.info(f'Minimum turning radius: {self.minimum_turning_radius}')
            if self.minimum_turning_radius * 2 > rows_distance:
                self.log.info('Rows distance is smaller than minimum turning radius * 2')
                minimum_row_distance = int(
                    np.ceil(self.minimum_turning_radius * 2 / rows_distance))

        self.log.info(f'Minimum row distance: {minimum_row_distance} need to skip {minimum_row_distance - 1} rows')
        if minimum_row_distance > 1:
            sequence = find_sequence(len(rows), minimum_distance=minimum_row_distance)
            if not sequence:
                self.log.warning('No sequence found')
                sequence = list(range(len(rows)))
        else:
            sequence = list(range(len(rows)))
        self.log.info(f'Row sequence: {sequence}')

        paths = []
        switch_first_row = False
        for i, row_index in enumerate(sequence):
            splines = []
            row = rows[row_index]
            self.sorted_weeding_rows.append(row)
            row_points = row.cartesian(reference).copy()
            if i == 0:
                switch_first_row = robot_position.distance(row_points[0]) > robot_position.distance(row_points[-1])
                self.log.info(f'Switch first row: {switch_first_row}')
            if not switch_first_row:
                if i % 2 != 0:
                    row_points = list(reversed(row_points))
            else:
                if i % 2 == 0:
                    row_points = list(reversed(row_points))
            self.log.info(f'Row {row.name} has {row_points} points')
            if row.crops:
                self.log.info(f'Row {row.name} has beets, creating {row.crops} points')
                # only take every tenth crop into account
                for i, beet in enumerate(row.crops):
                    if i % 10 == 0 and i != 0:
                        row_points.append(beet.position)
                row_points = sorted(row_points, key=lambda point: point.distance(row_points[0]))
                self.log.info(f'Row {row.name} has {len(row_points)} points')
            for j in range(len(row_points) - 1):
                splines.append(Spline.from_points(row_points[j], row_points[j + 1]))
            path = [PathSegment(spline=spline) for spline in splines]
            paths.append(path)
        return paths

    async def _generate_turn_paths(self) -> list[list[PathSegment]]:
        self.log.info('Generating turn paths...')
        if not self.weeding_plan or not self.field:
            self.log.error('No weeding plan or field available')
            return []
        turn_paths = []
        self.path_planner.obstacles.clear()
        assert self.field.reference is not None
        for obstacle in self.field.obstacles:
            self.path_planner.obstacles[obstacle.id] = \
                rosys.pathplanning.Obstacle(id=obstacle.id, outline=obstacle.cartesian(self.field.reference))
        for row in self.field.rows:
            row_points = row.cartesian(self.field.reference)
            # create a small polygon around the row to avoid the robot driving through the row
            row_polygon = [
                Point(x=row_points[0].x, y=row_points[0].y),
                Point(x=row_points[0].x - 0.01, y=row_points[0].y + 0.01),
                Point(x=row_points[-1].x, y=row_points[-1].y),
            ]
            self.path_planner.obstacles[f'row_{row.id}'] = \
                rosys.pathplanning.Obstacle(id=f'row_{row.id}', outline=row_polygon)

        area = rosys.pathplanning.Area(id=f'{self.field.id}', outline=self.field.outline)
        self.path_planner.areas = {area.id: area}
        for i in range(len(self.weeding_plan) - 1):
            # remove the current and the rows from obstacles to allow starting in it an insert it afterwards again
            start_row = self.sorted_weeding_rows[i]
            end_row = self.sorted_weeding_rows[i + 1]
            temp_removed_start_row = self.path_planner.obstacles.pop(f'row_{start_row.id}')
            temp_removed_end_row = self.path_planner.obstacles.pop(f'row_{end_row.id}')
            start_point = Point(x=self.weeding_plan[i][-1].spline.end.x,
                                y=self.weeding_plan[i][-1].spline.end.y)
            yaw = self.weeding_plan[i][-1].spline.start.direction(self.weeding_plan[i][-1].spline.end)
            offset_start_point = start_point.polar(self.turn_offset, yaw)

            start_pose = Pose(x=offset_start_point.x, y=offset_start_point.y, yaw=yaw)
            end_point = Point(x=self.weeding_plan[i + 1][0].spline.start.x,
                              y=self.weeding_plan[i + 1][0].spline.start.y)
            end_yaw = self.weeding_plan[i + 1][0].spline.start.direction(self.weeding_plan[i + 1][0].spline.end)
            offset_end_point = end_point.polar(0.5, yaw)
            end_pose = Pose(x=offset_end_point.x, y=offset_end_point.y, yaw=end_yaw)
            self.log.info(f'Searching path from row {i} to row {i + 1}...')
            turn_path = await self.path_planner.search(start=start_pose, goal=end_pose, timeout=120)
            if turn_path:
                turn_paths.append(turn_path)
            else:
                self.log.error(f'No turn path found from row {i} to row {i + 1}')
                return []
            self.path_planner.obstacles[f'row_{start_row.id}'] = temp_removed_start_row
            self.path_planner.obstacles[f'row_{end_row.id}'] = temp_removed_end_row
        # # clear all row obstacles
        # for row in self.field.rows:
        #     self.path_planner.obstacles.pop(f'row_{row.id}')
        return turn_paths

    async def _drive_to_start(self):
        self.log.info('Driving to start...')
        start_pose = self.odometer.prediction
        end_pose = Pose(x=self.weeding_plan[0][0].spline.start.x, y=self.weeding_plan[0][0].spline.start.y,
                        yaw=self.weeding_plan[0][0].spline.start.direction(self.weeding_plan[0][0].spline.end))
        start_spline = Spline.from_poses(start_pose, end_pose)
        await self.driver.drive_spline(start_spline)

    async def _drive_segment(self):
        self.log.info('Driving segment...')
        await self.driver.drive_spline(self.current_segment.spline)
        self.row_segment_completed = True

    def clear(self) -> None:
        super().clear()
        self.field = None
        self.start_row_id = None
        self.end_row_id = None
        self.sorted_weeding_rows = []
        self.weeding_plan = []
        self.turn_paths = []
        self.current_row = None
        self.current_segment = None
        self.row_segment_completed = False
        self.PATH_PLANNED.emit()

    def backup(self) -> dict:
        return {
            'start_row_id': self.start_row_id,
            'end_row_id': self.end_row_id,
            'minimum_turning_radius': self.minimum_turning_radius,
            'turn_offset': self.turn_offset,
            'sorted_weeding_rows': [rosys.persistence.to_dict(row) for row in self.sorted_weeding_rows],
            'field': rosys.persistence.to_dict(self.field) if self.field else None,
            'weeding_plan': [[rosys.persistence.to_dict(segment) for segment in row] for row in self.weeding_plan],
            'turn_paths': [rosys.persistence.to_dict(segment) for segment in self.turn_paths],
            'current_row': rosys.persistence.to_dict(self.current_row) if self.current_row else None,
            'current_segment': rosys.persistence.to_dict(self.current_segment) if self.current_segment else None,
        }

    def restore(self, data: dict[str, Any]) -> None:
        self.start_row_id = data.get('start_row_id', self.start_row_id)
        self.end_row_id = data.get('end_row_id', self.end_row_id)
        self.minimum_turning_radius = data.get('minimum_turning_radius', self.minimum_turning_radius)
        self.turn_offset = data.get('turn_offset', self.turn_offset)
        self.sorted_weeding_rows = [rosys.persistence.from_dict(Row, row_data)
                                    for row_data in data['sorted_weeding_rows']]
        self.field = rosys.persistence.from_dict(Field, data['field']) if data['field'] else None
        self.weeding_plan = [[rosys.persistence.from_dict(PathSegment, segment_data)
                              for segment_data in row_data] for row_data in data.get('weeding_plan', [])]
        self.turn_paths = [rosys.persistence.from_dict(PathSegment, segment_data)
                           for segment_data in data.get('turn_paths', [])]
        self.current_row = rosys.persistence.from_dict(Row, data['current_row']) if data['current_row'] else None
        self.current_segment = rosys.persistence.from_dict(PathSegment, data['current_segment']) \
            if data['current_segment'] else None
