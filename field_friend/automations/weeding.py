import logging
from typing import TYPE_CHECKING, Any, Optional

import numpy as np
import rosys
from rosys.driving import PathSegment
from rosys.geometry import Point, Pose, Spline
from rosys.helpers import eliminate_2pi

from ..hardware import ChainAxis
from .field_provider import Field, Row
from .plant_provider import Plant
from .sequence import find_sequence

if TYPE_CHECKING:
    from system import System


class WorkflowException(Exception):
    pass


class Weeding(rosys.persistence.PersistentModule):
    def __init__(self, system: 'System') -> None:
        super().__init__()
        self.PATH_PLANNED = rosys.event.Event()
        '''Event that is emitted when the path is planed. The event contains the path as a list of PathSegments.'''

        self.log = logging.getLogger('field_friend.weeding')
        self.system = system
        self.kpi_provider = system.kpi_provider

        # default settings
        self.continue_canceled_weeding: bool = False
        self.use_monitor_workflow: bool = False

        # field settings
        self.use_field_planning = True
        self.field: Optional[Field] = None
        self.start_row_id: Optional[str] = None
        self.end_row_id: Optional[str] = None
        self.minimum_turning_radius: float = 0.5

        # workflow settings
        self.only_monitoring: bool = False

        # tool settings
        self.tornado_angle: float = 110.0
        self.weed_screw_depth: float = 0.15
        self.crop_safety_distance: float = 0.01

        # driver settings

        self.sorted_weeding_rows: list = []
        self.weeding_plan: Optional[list[list[PathSegment]]] = None
        self.turn_paths: list[list[PathSegment]] = []
        self.current_row: Optional[Row] = None
        self.current_segment: Optional[PathSegment] = None
        self.row_segment_completed: bool = False
        self.crops_to_handle: dict[str, Point] = {}
        self.weeds_to_handle: dict[str, Point] = {}

    def backup(self) -> dict:
        return {
            'use_field_planning': self.use_field_planning,
            'start_row_id': self.start_row_id,
            'end_row_id': self.end_row_id,
            'tornado_angle': self.tornado_angle,
            'minimum_turning_radius': self.minimum_turning_radius,
            'only_monitoring': self.only_monitoring,
            'sorted_weeding_rows': self.sorted_weeding_rows,
            'weeding_plan': [[rosys.persistence.to_dict(segment) for segment in row] for row in self.weeding_plan] if self.weeding_plan else [],
            'turn_paths': [rosys.persistence.to_dict(segment) for segment in self.turn_paths],
            'current_row': rosys.persistence.to_dict(self.current_row) if self.current_row else None,
            'current_segment': rosys.persistence.to_dict(self.current_segment) if self.current_segment else None,
        }

    def restore(self, data: dict[str, Any]) -> None:
        self.use_field_planning = data.get('use_field_planning', False)
        self.start_row_id = data.get('start_row_id')
        self.end_row_id = data.get('end_row_id')
        self.tornado_angle = data.get('tornado_angle', 110.0)
        self.minimum_turning_radius = data.get('minimum_turning_radius', 0.5)
        self.only_monitoring = data.get('only_monitoring', False)
        self.sorted_weeding_rows = data.get('sorted_weeding_rows', [])
        self.weeding_plan = [
            [rosys.persistence.from_dict(PathSegment, segment_data)
             for segment_data in row_data] for row_data in data.get('weeding_plan', [])
        ]
        self.turn_paths = [rosys.persistence.from_dict(PathSegment, segment_data)
                           for segment_data in data.get('turn_paths', [])]
        self.current_row = rosys.persistence.from_dict(
            Row, data['current_row']) if data['current_row'] else None
        self.current_segment = rosys.persistence.from_dict(PathSegment, data[
            'current_segment']) if data['current_segment'] else None

    def invalidate(self) -> None:
        self.request_backup()

    async def start(self):
        self.log.info('starting weeding...')
        if not await self._check_hardware_ready():
            return
        if not self.continue_canceled_weeding:
            if self.use_field_planning and not await self._field_planning():
                rosys.notify('Field planning failed', 'negative')
                return
        await self._weeding()

    async def _check_hardware_ready(self) -> bool:
        if self.system.field_friend.estop.active or self.system.field_friend.estop.is_soft_estop_active:
            rosys.notify('E-Stop is active, aborting', 'negative')
            self.log.error('E-Stop is active, aborting')
            return False
        camera = next((camera for camera in self.system.usb_camera_provider.cameras.values() if camera.is_connected), None)
        if not camera:
            rosys.notify('no camera connected')
            return False
        if camera.calibration is None:
            rosys.notify('camera has no calibration')
            return False
        if self.use_monitor_workflow:
            return True
        if self.system.field_friend.tool == 'none':
            rosys.notify('This field friend has no tool, only monitoring', 'info')
            self.log.info('This field friend has no tool, only monitoring')
            return True
        if self.system.field_friend.y_axis.alarm:
            rosys.notify('Y-Axis is in alarm, aborting', 'negative')
            self.log.error('Y-Axis is in alarm, aborting')
            return False
        if isinstance(self.system.field_friend.y_axis, ChainAxis):
            if not self.system.field_friend.y_axis.ref_t:
                rosys.notify('ChainAxis is not in top ref', 'negative')
                self.log.error('ChainAxis is not in top ref')
                return False
        if not await self.system.puncher.try_home():
            rosys.notify('Puncher homing failed, aborting', 'negative')
            self.log.error('Puncher homing failed, aborting')
            return False
        return True

    async def _field_planning(self) -> bool:
        if self.system.gnss.device is None:
            self.log.error('GNSS is not available')
            return False
        self.field = self.system.field_provider.active_field
        if self.field is None:
            self.log.error('Field is not available')
            rosys.notify('No field selected', 'negative')
            return False
        if not self.field.reference_lat or not self.field.reference_lon:
            self.log.error('Field reference is not available')
            return False
        self.system.gnss.set_reference(self.field.reference_lat, self.field.reference_lon)
        self.weeding_plan = self._make_plan()
        if not self.weeding_plan:
            self.log.error('No plan available')
            return False
        self.turn_paths = await self._generate_turn_paths()
        if not self.turn_paths:
            self.log.error('No turn paths available')
            return False
        self.log.info(f'Turn paths: {self.turn_paths}')
        self.log.info(
            f'Planned path: {[path_segment for path in self.weeding_plan for path_segment in path] + self.turn_paths}')
        paths = [path_segment for path in self.weeding_plan for path_segment in path]
        turn_paths = [path_segment for path in self.turn_paths for path_segment in path]
        self.PATH_PLANNED.emit(paths + turn_paths)
        self.invalidate()
        return True

    def _make_plan(self) -> Optional[list[list[rosys.driving.PathSegment]]]:
        self.log.info('Making plan...')
        if self.field is None:
            self.log.warning('No field available')
            return None
        if not self.field.rows:
            self.log.warning('No rows available')
            return None
        if self.start_row_id is None:
            self.start_row_id = self.field.rows[0].id
        else:
            self.start_row_id = next((row.id for row in self.field.rows if row.id == self.start_row_id), None)
        if self.end_row_id is None:
            self.end_row_id = self.field.rows[-1].id
        else:
            self.end_row_id = next((row.id for row in self.field.rows if row.id == self.end_row_id), None)

        start_row = next((row for row in self.field.rows if row.id == self.start_row_id), None)
        end_row = next((row for row in self.field.rows if row.id == self.end_row_id), None)
        if start_row is None or end_row is None:
            self.log.warning('Start or end row not available')
            return None
        reference = [self.field.reference_lat, self.field.reference_lon]
        rows_to_weed = self.field.rows[self.field.rows.index(
            start_row):self.field.rows.index(end_row) + 1]
        rows = [row for row in rows_to_weed if len(row.points(reference)) > 1]
        robot_position = self.system.odometer.prediction.point
        distance_to_first_row = min([point.distance(robot_position) for point in rows[0].points(reference)])
        distance_to_last_row = min([point.distance(robot_position) for point in rows[-1].points(reference)])
        if distance_to_first_row > distance_to_last_row:
            rows = list(reversed(rows))
        minimum_row_distance = 1  # 1 = no row needs to be skippen when turning
        if len(rows) > 1:
            rows_distance = rows[0].points(reference)[0].distance(rows[1].points(reference)[0])
            if self.minimum_turning_radius * 2 > rows_distance:
                minimum_row_distance = int(
                    np.ceil(self.minimum_turning_radius * 2 / rows_distance))

        self.log.info(f'Minimum turning distance: {minimum_row_distance}')
        if minimum_row_distance > 1:
            sequence = find_sequence(len(rows), minimum_distance=minimum_row_distance)
            if not sequence:
                self.log.warning('No sequence found')
                return None
                # sequence = list(range(len(rows)))
        else:
            sequence = list(range(len(rows)))
        self.log.info(f'Sequence: {sequence}')

        paths = []
        switch_first_row = False
        for i, row_index in enumerate(sequence):
            splines = []
            row = rows[row_index]
            self.sorted_weeding_rows.append(row)
            row_points = row.points(reference).copy()
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
        self.system.path_planner.obstacles.clear()
        for obstacle in self.field.obstacles:
            self.system.path_planner.obstacles[obstacle.id] = rosys.pathplanning.Obstacle(
                id=obstacle.id, outline=obstacle.points([self.field.reference_lat, self.field.reference_lon]))
        for row in self.field.rows:
            row_points = row.points([self.field.reference_lat, self.field.reference_lon])
            # create a small polygon around the row to avoid the robot driving through the row
            row_polygon = [
                Point(x=row_points[0].x - 0.01, y=row_points[0].y - 0.01),
                Point(x=row_points[0].x - 0.01, y=row_points[-1].y + 0.01),
                Point(x=row_points[-1].x + 0.01, y=row_points[-1].y + 0.01),
                Point(x=row_points[-1].x + 0.01, y=row_points[0].y - 0.01),
            ]
            self.system.path_planner.obstacles[f'row_{row.id}'] = rosys.pathplanning.Obstacle(
                id=f'row_{row.id}', outline=row_polygon)

        area = rosys.pathplanning.Area(id=f'{self.field.id}', outline=self.field.outline)
        self.system.path_planner.areas = {area.id: area}
        for i in range(len(self.weeding_plan) - 1):
            # remove this rows from obstacles to allow starting in it an insert it afterwards again
            start_row = self.sorted_weeding_rows[i]
            end_row = self.sorted_weeding_rows[i + 1]
            temp_removed_start_row = self.system.path_planner.obstacles.pop(f'row_{start_row.id}')
            temp_removed_end_row = self.system.path_planner.obstacles.pop(f'row_{end_row.id}')
            start_pose = Pose(x=self.weeding_plan[i][-1].spline.end.x,
                              y=self.weeding_plan[i][-1].spline.end.y,
                              yaw=self.weeding_plan[i][-1].spline.start.direction(self.weeding_plan[i][-1].spline.end))
            end_pose = Pose(x=self.weeding_plan[i + 1][0].spline.start.x,
                            y=self.weeding_plan[i + 1][0].spline.start.y,
                            yaw=self.weeding_plan[i + 1][0].spline.start.direction(self.weeding_plan[i + 1][0].spline.end))
            self.log.info(f'Searching path from row {i} to row {i + 1}...')
            turn_path = await self.system.path_planner.search(start=start_pose, goal=end_pose, timeout=120)
            if turn_path:
                turn_paths.append(turn_path)
            else:
                self.log.error(f'No turn path found from row {i} to row {i + 1}')
                return []
            self.system.path_planner.obstacles[f'row_{start_row.id}'] = temp_removed_start_row
            self.system.path_planner.obstacles[f'row_{end_row.id}'] = temp_removed_end_row
        return turn_paths

    async def _weeding(self):
        self.log.info('Starting driving...')
        await rosys.sleep(0.5)
        try:
            if self.weeding_plan:
                await self._weed_with_plan()
                self.log.info('Weeding with plan completed')
            else:
                await self._weed_planless()
                self.log.info('Planless weeding completed')

        except WorkflowException as e:
            self.kpi_provider.increment('automation_stopped')
            self.log.error(f'WorkflowException: {e}')
        finally:
            self.kpi_provider.increment_weeding_kpi('weeding_completed')
            await self.system.field_friend.stop()
            self.system.plant_locator.pause()
            self.system.automation_watcher.stop_field_watch()
            self.system.automation_watcher.gnss_watch_active = False

    async def _drive_to_start(self):
        self.log.info('Driving to start...')
        start_pose = self.system.odometer.prediction
        end_pose = Pose(x=self.weeding_plan[0][0].spline.start.x, y=self.weeding_plan[0][0].spline.start.y,
                        yaw=self.weeding_plan[0][0].spline.start.direction(self.weeding_plan[0][0].spline.end))
        start_spline = Spline.from_poses(start_pose, end_pose)
        await self.system.driver.drive_spline(start_spline)

    async def _weed_with_plan(self):
        if self.continue_canceled_weeding:
            start_pose = self.system.odometer.prediction
            end_pose = Pose(x=self.current_segment.spline.start.x, y=self.current_segment.spline.start.y,
                            yaw=self.current_segment.spline.start.direction(self.current_segment.spline.end))
            start_spline = Spline.from_poses(start_pose, end_pose)
            await self.system.driver.drive_spline(start_spline)
        else:
            await self._drive_to_start()
        self.system.automation_watcher.start_field_watch(self.field.outline)
        self.system.automation_watcher.gnss_watch_active = True
        for i, path in enumerate(self.weeding_plan):
            if self.continue_canceled_weeding and self.current_row != self.sorted_weeding_rows[i]:
                continue
            self.system.driver.parameters.can_drive_backwards = False
            self.system.driver.parameters.minimum_turning_radius = 0.02
            self.current_row = self.sorted_weeding_rows[i]
            self.system.plant_locator.pause()
            self.system.plant_provider.clear()
            if self.system.field_friend.tool != 'none':
                await self.system.puncher.clear_view()
            await self.system.field_friend.flashlight.turn_on()
            await rosys.sleep(3)
            self.system.plant_locator.resume()
            await rosys.sleep(3)
            for j, segment in enumerate(path):
                if self.continue_canceled_weeding and self.current_segment != segment:
                    continue
                else:
                    self.continue_canceled_weeding = False
                self.current_segment = segment
                self.invalidate()
                if not self.system.is_real:
                    self._create_simulated_plants()
                self.log.info(f'Driving row {i + 1}/{len(self.weeding_plan)} and segment {j + 1}/{len(path)}...')
                self.row_segment_completed = False
                while not self.row_segment_completed:
                    self.log.info('while not row completed...')
                    await rosys.automation.parallelize(
                        self._check_for_plants(),
                        self._drive_segment(),
                        return_when_first_completed=True
                    )
                    if self.crops_to_handle or self.weeds_to_handle:
                        self.log.info('Plants to handle...')
                        await self._handle_plants()
                        self.crops_to_handle = {}
                        self.weeds_to_handle = {}

            await self.system.field_friend.flashlight.turn_off()
            self.system.plant_locator.pause()
            if i < len(self.weeding_plan) - 1:
                self.system.driver.parameters.can_drive_backwards = True
                self.log.info('Driving to next row...')
                turn_path = self.turn_paths[i]
                await self.system.driver.drive_path(turn_path)
                await rosys.sleep(1)
            self.kpi_provider.increment_weeding_kpi('rows_weeded')

        self.system.automation_watcher.stop_field_watch()
        self.system.automation_watcher.gnss_watch_active = False
        self.sorted_weeding_rows = []
        self.weeding_plan = None
        self.turn_paths = []
        self.current_row = None
        self.current_segment = None

    async def _weed_planless(self):
        already_explored_count = 0
        while True:
            self.system.plant_locator.pause()
            self.system.plant_provider.clear()
            if not self.system.is_real:
                self.system.detector.simulated_objects = []
                self._create_simulated_plants()
            if self.system.field_friend.tool != 'none':
                await self.system.puncher.clear_view()
            await self.system.field_friend.flashlight.turn_on()
            await rosys.sleep(2)
            self.system.plant_locator.resume()
            await rosys.sleep(0.5)
            await self._get_upcoming_crops()
            while self.crops_to_handle or self.weeds_to_handle:
                await self._handle_plants()
                already_explored_count = 0
                await rosys.sleep(0.2)
                await self._get_upcoming_crops()
            if not self.crops_to_handle and already_explored_count != 5:
                self.log.info('No crops found, advancing a bit to ensure there are really no more crops')
                target = self.system.odometer.prediction.transform(Point(x=0.10, y=0))
                await self.system.driver.drive_to(target)
                already_explored_count += 1
            else:
                self.log.info('No more crops found')
                break

    async def _drive_segment(self):
        self.log.info('Driving segment...')
        await self.system.driver.drive_spline(self.current_segment.spline)
        self.row_segment_completed = True

    async def _check_for_plants(self):
        self.log.info('Checking for plants...')
        while True:
            await self._get_upcoming_crops()
            if self.system.field_friend.tool == 'tornado':
                if self.crops_to_handle:
                    return
            if self.weeds_to_handle or self.crops_to_handle:
                return
            await rosys.sleep(0.2)

    async def _get_upcoming_crops(self):
        relative_crop_positions = {
            c.id: self.system.odometer.prediction.relative_point(c.position)
            for c in self.system.plant_provider.crops if c.position.distance(self.system.odometer.prediction.point) < 0.5
        }
        # remove very distant crops (probably not row
        if self.current_segment:
            # Correctly filter to get upcoming crops based on their x position
            upcoming_crop_positions = {
                c: pos for c, pos in relative_crop_positions.items()
                if self.system.field_friend.WORK_X + self.system.field_friend.DRILL_RADIUS < pos.x <= self.system.odometer.prediction.relative_point(self.current_segment.spline.end).x
            }
            self.log.info(f'Upcoming crops in segment: {upcoming_crop_positions}')
        else:
            upcoming_crop_positions = {
                c: pos for c, pos in relative_crop_positions.items()
                if self.system.field_friend.WORK_X + self.system.field_friend.DRILL_RADIUS < pos.x
            }

        # Sort the upcoming_crop_positions dictionary by the .x attribute of its values
        sorted_crops = dict(sorted(upcoming_crop_positions.items(), key=lambda item: item[1].x))

        self.crops_to_handle = sorted_crops

        relative_weed_positions = {
            w.id: self.system.odometer.prediction.relative_point(w.position)
            for w in self.system.plant_provider.weeds
        }
        if self.current_segment:
            # Filter to get upcoming weeds based on their .x position
            upcoming_weed_positions = {
                w: pos for w, pos in relative_weed_positions.items()
                if self.system.field_friend.WORK_X + self.system.field_friend.DRILL_RADIUS < pos.x <= self.current_segment.spline.end.x
            }
        else:
            upcoming_weed_positions = {
                w: pos for w, pos in relative_weed_positions.items()
                if self.system.field_friend.WORK_X + self.system.field_friend.DRILL_RADIUS < pos.x
            }

        # Sort the upcoming_weed_positions dictionary by the .x attribute of its values
        sorted_weeds = dict(sorted(upcoming_weed_positions.items(), key=lambda item: item[1].x))

        self.weeds_to_handle = sorted_weeds

    async def _handle_plants(self) -> None:
        self.log.info('Handling plants...')
        for crop_id in self.crops_to_handle:
            self._safe_crop_to_row(crop_id)

        if self.system.field_friend.tool == 'tornado' and self.crops_to_handle and not self.use_monitor_workflow:
            await self._tornado_workflow()
        elif self.system.field_friend.tool == 'weed_screw':
            await self._weed_screw_workflow()
        elif self.system.field_friend.tool == 'none' or self.use_monitor_workflow:
            await self._monitor_workflow()

        # ToDo: implement workflow of other tools

    async def _tornado_workflow(self) -> None:
        self.log.info('Starting Tornado Workflow..')
        try:
            closest_crop_position = list(self.crops_to_handle.values())[0]
            self.log.info(f'Closest crop position: {closest_crop_position}')
            # fist check if the closest crop is in the working area
            if closest_crop_position.x < self.system.field_friend.WORK_X + 0.05:
                self.log.info(f'target next crop at {closest_crop_position}')
                # do not steer while advancing on a crop
                drive_distance = closest_crop_position.x - self.system.field_friend.WORK_X
                target = self.system.odometer.prediction.transform(Point(x=drive_distance, y=0))
                await self.system.driver.drive_to(target)
                if not self.only_monitoring and self.system.field_friend.can_reach(closest_crop_position):
                    await self.system.puncher.punch(closest_crop_position.y, angle=self.tornado_angle)
                if len(self.crops_to_handle) > 1:
                    self.log.info('checking for second closest crop')
                    second_closest_crop_position = list(self.crops_to_handle.values())[1]
                    distance_to_next_crop = closest_crop_position.distance(second_closest_crop_position)
                    if distance_to_next_crop > 0.15:
                        # get the target of half the distance between the two crops
                        self.log.info(f'driving to position between two crops: {target}')
                        target = self.system.odometer.prediction.transform(
                            Point(x=closest_crop_position.x + distance_to_next_crop / 2 - self.system.field_friend.WORK_X, y=0))
                        await self.system.driver.drive_to(target)
                        self.log.info(f'target between two crops at {target}')
                        if not self.only_monitoring:
                            # punch in the middle position with closed knives
                            await self.system.puncher.punch(0, angle=180)

                self.system.plant_locator.resume()
            else:
                self.log.info('follow line of crops')
                farthest_crop = list(self.crops_to_handle.values())[-1]
                self.log.info(f'Farthest crop: {farthest_crop}')
                upcoming_world_position = self.system.odometer.prediction.transform(farthest_crop)
                yaw = self.system.odometer.prediction.point.direction(upcoming_world_position)
                # only apply minimal yaw corrections to avoid oversteering
                yaw = eliminate_2pi(self.system.odometer.prediction.yaw) * 0.85 + eliminate_2pi(yaw) * 0.15
                target = self.system.odometer.prediction.point.polar(0.04, yaw)

                self.log.info(f'current world position: {self.system.odometer.prediction} target next crop at {target}')
                await self.system.driver.drive_to(target)
            await rosys.sleep(0.2)
            self.log.info('workflow completed')
        except Exception as e:
            raise WorkflowException(f'Error while tornado Workflow: {e}') from e

    async def _monitor_workflow(self) -> None:
        self.log.info('Starting Monitoring Workflow...')
        try:
            closest_crop_position = list(self.crops_to_handle.values())[0]
            self.log.info(f'Closest crop position: {closest_crop_position}')
            # fist check if the closest crop is in the working area
            if closest_crop_position.x < 0.05:
                self.log.info(f'target next crop at {closest_crop_position}')
                # do not steer while advancing on a crop
                target = self.system.odometer.prediction.transform(Point(x=closest_crop_position.x, y=0))
                await self.system.driver.drive_to(target)
                self.system.plant_locator.resume()
            else:
                if self.crops_to_handle:
                    self.log.info('follow line of crops')
                    farthest_crop = list(self.crops_to_handle.values())[-1]
                    self.log.info(f'Farthest crop: {farthest_crop}')
                    upcoming_world_position = self.system.odometer.prediction.transform(farthest_crop)
                    yaw = self.system.odometer.prediction.point.direction(upcoming_world_position)
                    # only apply minimal yaw corrections to avoid oversteering
                    yaw = eliminate_2pi(self.system.odometer.prediction.yaw) * 0.85 + eliminate_2pi(yaw) * 0.15
                    target = self.system.odometer.prediction.point.polar(0.04, yaw)
                else:
                    target = self.system.odometer.prediction.point.polar(0.04, self.system.odometer.prediction.yaw)
                self.log.info(f'current world position: {self.system.odometer.prediction} target next crop at {target}')
                await self.system.driver.drive_to(target)
            await rosys.sleep(0.2)
            self.log.info('workflow completed')
        except Exception as e:
            raise WorkflowException(f'Error while Monitoring Workflow: {e}') from e

    async def _weed_screw_workflow(self) -> None:
        self.log.info('Starting Weed Screw Workflow...')
        try:
            self._keep_crops_safe()
            next_weed_position = None
            if self.weeds_to_handle:
                next_weed_position = list(self.weeds_to_handle.values())[0]
                self.log.info(f'Next weed position: {next_weed_position}')

            if next_weed_position and next_weed_position.x < self.system.field_friend.WORK_X + 0.06:
                self.log.info(f'target next weed at {next_weed_position}')
                drive_distance = next_weed_position.x - self.system.field_friend.WORK_X
                target = self.system.odometer.prediction.transform(
                    Point(x=drive_distance, y=0))
                await self.system.driver.drive_to(target)
                corrected_positions = [Point(x=position.x - drive_distance, y=position.y)
                                       for _, position in self.weeds_to_handle.items()]
                for index, position in enumerate(corrected_positions):
                    if self.system.field_friend.can_reach(position):
                        if not self.only_monitoring:
                            await self.system.puncher.punch(position.y, depth=self.weed_screw_depth)
                            corrected_positions.pop(index)
                        self.log.info(f'Punched weed at {position}')
                        for index, other_position in enumerate(corrected_positions):
                            if position.distance(other_position) < self.system.field_friend.DRILL_RADIUS:
                                corrected_positions.pop(index)
                                self.log.info(f'Punched weed at {other_position}')
                    else:
                        self.log.info(f'Could not reach weed at {position}')
                self.system.plant_locator.resume()
            elif self.crops_to_handle:
                self.log.info('follow line of crops')
                farthest_crop = list(self.crops_to_handle.values())[-1]
                self.log.info(f'Farthest crop: {farthest_crop}')
                upcoming_world_position = self.system.odometer.prediction.transform(farthest_crop)
                yaw = self.system.odometer.prediction.point.direction(upcoming_world_position)
                # only apply minimal yaw corrections to avoid oversteering
                yaw = eliminate_2pi(self.system.odometer.prediction.yaw) * 0.85 + eliminate_2pi(yaw) * 0.15
                target = self.system.odometer.prediction.point.polar(0.04, yaw)

                self.log.info(f'current world position: {self.system.odometer.prediction} target next crop at {target}')
                await self.system.driver.drive_to(target)
            else:
                self.log.info('No crops and no weeeds in range, driving forward a bit...')
                target = self.system.odometer.prediction.point.polar(0.04, self.system.odometer.prediction.yaw)
                self.log.info(f'current world position: {self.system.odometer.prediction} target: {target}')
                await self.system.driver.drive_to(target)

            await rosys.sleep(0.2)
            self.log.info('workflow completed')
        except Exception as e:
            raise WorkflowException(f'Error while Weed Screw Workflow: {e}') from e

    def _keep_crops_safe(self) -> None:
        self.log.info('Keeping crops safe...')
        for crop, crop_position in self.crops_to_handle.items():
            if crop_position.x > 0.06:
                continue
            for weed, weed_position in self.weeds_to_handle.items():
                if weed_position.x > 0.06:
                    continue
                offset = self.system.field_friend.DRILL_RADIUS + \
                    self.crop_safety_distance - crop_position.distance(weed_position)
                if offset > 0:
                    safe_weed_position = weed_position.polar(offset, crop_position.direction(weed_position))
                    self.weeds_to_handle[weed] = safe_weed_position
                    self.log.info(f'Moved weed {weed} to {safe_weed_position} to safe {crop} at {crop_position}')

    def _safe_crop_to_row(self, crop_id: str) -> None:
        if self.current_row is None:
            return
        crop = next((c for c in self.system.plant_provider.crops if c.id == crop_id), None)
        if crop is None:
            self.log.error(f'Error in crop saving: Crop with id {crop_id} not found')
            return
        for c in self.current_row.crops:
            if c.position.distance(crop.position) < 0.08 and c.type == crop.type:
                if c.confidence >= crop.confidence:
                    return
                self.log.info('Updating crop with higher confidence')
                c.position = crop.position
                return
        if crop.confidence >= 0.9:
            self.log.info('Adding new crop to row')
            self.current_row.crops.append(crop)

    def _create_simulated_plants(self):
        self.log.info('Creating simulated plants...')
        if self.current_segment:
            first_point = self.current_segment.spline.start
            last_point = self.current_segment.spline.end
            distance = self.current_segment.spline.start.distance(self.current_segment.spline.end)
            for i in range(1, int(distance/0.20)):
                self.system.plant_provider.add_crop(Plant(
                    id=str(i),
                    type='beet',
                    position=first_point.polar(0.20*i, first_point.direction(last_point)),
                    detection_time=rosys.time(),
                    confidence=0.9,
                ))
                for j in range(0, 2):
                    self.system.plant_provider.add_weed(Plant(
                        id=str(i),
                        type='weed',
                        position=first_point.polar(0.20*i, first_point.direction(last_point)
                                                   ).polar(0.04*j, first_point.direction(last_point) + 1.57),
                        detection_time=rosys.time(),
                        confidence=0.9,
                    ))
                for j in range(0, 2):
                    self.system.plant_provider.add_weed(Plant(
                        id=str(i),
                        type='weed',
                        position=first_point.polar(0.20*i, first_point.direction(last_point)
                                                   ).polar(0.02*j, first_point.direction(last_point) - 1.57),
                        detection_time=rosys.time(),
                        confidence=0.9,
                    ))
        else:
            for i in range(0, 30):
                self.system.plant_provider.add_crop(Plant(
                    id=str(i),
                    type='beet',
                    position=self.system.odometer.prediction.point.polar(0.20*i, self.system.odometer.prediction.yaw),
                    detection_time=rosys.time(),
                    confidence=0.9,
                ))
            for i in range(0, 30):
                for j in range(0, 2):
                    self.system.plant_provider.add_weed(Plant(
                        id=str(i),
                        type='weed',
                        position=self.system.odometer.prediction.point.polar(
                            0.20*i, self.system.odometer.prediction.yaw).polar(0.02*j, self.system.odometer.prediction.yaw + 1.57),
                        detection_time=rosys.time(),
                        confidence=0.9,
                    ))
                for j in range(0, 2):
                    self.system.plant_provider.add_weed(Plant(
                        id=str(i),
                        type='weed',
                        position=self.system.odometer.prediction.point.polar(
                            0.20*i, self.system.odometer.prediction.yaw).polar(0.02*j, self.system.odometer.prediction.yaw - 1.57),
                        detection_time=rosys.time(),
                        confidence=0.9,
                    ))
