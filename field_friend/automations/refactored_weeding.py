import logging
from typing import TYPE_CHECKING, Optional

import numpy as np
import rosys
from rosys import persistence
from rosys.driving import PathSegment
from rosys.geometry import Point, Pose, Spline

from ..hardware import ChainAxis
from .field_provider import Field, Row
from .plant_provider import Plant
from .sequence import find_sequence

if TYPE_CHECKING:
    from system import System


class WorkflowException(Exception):
    pass


class Weeding:
    def __init__(self, system: 'System') -> None:
        super().__init__()
        self.log = logging.getLogger('field_friend.weeding')
        self.system = system
        self.with_field_planning = False

        self.field: Optional[Field] = None
        self.start_row: Optional[Row] = None
        self.end_row: Optional[Row] = None
        self.tornado_angle: float = 110.0

        self.ordered_rows: list = []
        self.plan: Optional[list[list[PathSegment]]] = None
        self.current_row: Optional[Row] = None
        self.current_segment: Optional[PathSegment] = None
        self.row_segment_completed: bool = False
        self.crops_to_handle: dict[Plant, Point] = {}
        self.weeds_to_handle: dict[Plant, Point] = {}

    async def start(self):
        self.log.info('starting weeding...')
        if not await self._check_hardware_ready():
            return
        if self.with_field_planning and not await self._field_planning():
            return
        await self._weeding()

    async def _check_hardware_ready(self) -> bool:
        if self.system.field_friend.estop.active or self.system.field_friend.estop.is_soft_estop_active:
            rosys.notify('E-Stop is active, aborting', 'negative')
            self.log.error('E-Stop is active, aborting')
            return False
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
            rosys.notify('Puncher homing failed, aborting', 'error')
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
            return False
        if not self.field.reference_lat or not self.field.reference_lon:
            self.log.error('Field reference is not available')
            return False
        self.system.gnss.set_reference(self.field.reference_lat, self.field.reference_lon)
        # ToDo: implement check if robot in field
        self.plan = self._make_plan()
        if not self.plan:
            self.log.error('No plan available')
            return False

        return True

    def _make_plan(self) -> Optional[list[rosys.driving.PathSegment]]:
        self.log.info('Making plan...')
        if self.field is None:
            self.log.warning('No field available')
            return None
        if not self.field.rows:
            self.log.warning('No rows available')
            return None
        if self.start_row is None:
            self.start_row = self.field.rows[0]
        if self.end_row is None:
            self.end_row = self.field.rows[-1]
            reference = [self.field.reference_lat, self.field.reference_lon]
        rows_to_weed = self.field.rows[self.field.rows.index(self.start_row):self.field.rows.index(self.end_row) + 1]
        rows = [row for row in rows_to_weed if len(row.points(reference)) > 1]
        robot_position = self.system.odometer.prediction.point
        distance_to_first_row = min([point.distance(robot_position) for point in rows[0].points(reference)])
        distance_to_last_row = min([point.distance(robot_position) for point in rows[-1].points(reference)])
        if distance_to_first_row > distance_to_last_row:
            rows = list(reversed(rows))
        minimum_turning_distance = 1  # 1 = no row needs to be skippen when turning
        if len(rows) > 1:
            rows_distance = rows[0].points(reference)[0].distance(rows[1].points(reference)[0])
            if self.system.driver.parameters.minimum_turning_radius * 2 > rows_distance:
                minimum_turning_distance = int(
                    np.ceil(self.system.driver.parameters.minimum_turning_radius * 2 / rows_distance))

        self.log.info(f'Minimum turning distance: {minimum_turning_distance}')
        if minimum_turning_distance > 1:
            sequence = find_sequence(len(rows), minimum_distance=minimum_turning_distance)
            if not sequence:
                self.log.warning('No sequence found')
                return None
                # sequence = list(range(len(rows)))
        else:
            sequence = list(range(len(rows)))

        paths = []
        for i, row_index in enumerate(sequence):
            splines = []
            row = rows[row_index]
            self.ordered_rows.append(row)
            row_points = row.points(reference).copy()
            switch_first_row = False
            if i == 0:
                switch_first_row = robot_position.distance(row_points[0]) > robot_position.distance(row_points[-1])
            if not switch_first_row:
                if i % 2 != 0:
                    row_points = list(reversed(row_points))
            else:
                if i % 2 == 0:
                    row_points = list(reversed(row_points))
            self.log.info(f'Row {row.id} has {row_points} points')
            if row.crops:
                self.log.info(f'Row {row.id} has beets, creating {len(row.crops)} points')
                for beet in row.crops:
                    row_points.append(beet.position)
                row_points = sorted(row_points, key=lambda point: point.distance(row_points[0]))
                self.log.info(f'Row {row.id} has {row_points} points')
            for j in range(len(row_points) - 1):
                splines.append(Spline.from_points(row_points[j], row_points[j + 1]))
            path = [PathSegment(spline=spline) for spline in splines]
            paths.append(path)
        return paths

    async def _weeding(self):
        self.log.info('Starting driving...')
        self.system.plant_locator.pause()
        self.system.plant_provider.clear()
        # if not self.system.is_real:
        #     self.create_simulated_plants()
        self.system.plant_locator.resume()
        await rosys.sleep(0.5)
        try:
            if self.plan:
                await self._weed_with_plan()
            else:
                await self._weed_planless()

        except WorkflowException as e:
            self.log.error(f'WorkflowException: {e}')
        finally:
            await self.system.field_friend.stop()

    async def _drive_to_start(self):
        self.log.info('Driving to start...')
        start_pose = self.system.odometer.prediction
        end_pose = Pose(x=self.plan[0][0].spline.start.x, y=self.plan[0][0].spline.start.y,
                        yaw=self.plan[0][0].spline.start.direction(self.plan[0][0].spline.end))
        start_spline = Spline.from_poses(start_pose, end_pose)
        await self.system.driver.drive_spline(start_spline)

    async def _weed_with_plan(self):
        await self._drive_to_start()
        for i, path in enumerate(self.plan):
            self.current_row = self.ordered_rows[i]
            for j, segment in enumerate(path):
                self.current_segment = segment
                self.log.info(f'Driving row {i + 1}/{len(self.plan)} and segment {j + 1}/{len(path)}...')
                self.row_segment_completed = False
                # counter = 0  # for simulation
                await self.system.puncher.clear_view()
                await self.system.field_friend.flashlight.turn_on()
                await rosys.sleep(3)
                while not self.row_segment_completed:
                    self.log.info('while not row completed...')
                    await rosys.automation.parallelize(
                        self._drive_segment(),
                        self._check_for_plants(),
                        return_when_first_completed=True
                    )
                    if self.crops_to_handle or self.weeds_to_handle:
                        self.log.info('Plants to handle...')
                        await self._handle_plants()
                        self.crops_to_handle = {}
                        self.weeds_to_handle = {}

                await self.system.field_friend.flashlight.turn_off()
                if i < len(self.plan) - 1:
                    self.log.info('Driving to next row...')
                    turn_path = self._generate_turn_path(self.current_segment.spline, self.plan[i + 1][0].spline)
                    await self.system.driver.drive_path(turn_path)

    async def _weed_planless(self):
        pass
        # ToDo: implement planless weeding

    async def _drive_segment(self):
        self.log.info('Driving segment...')
        await self.system.driver.drive_spline(self.current_segment.spline)
        self.row_segment_completed = True

    async def _check_for_plants(self):
        self.log.info('Checking for plants...')
        while True:
            await self._get_upcoming_crops()
            if self.weeds_to_handle or self.crops_to_handle:
                return
            await rosys.sleep(0.2)

    async def _get_upcoming_crops(self):
        relative_crop_positions = {
            c: self.system.odometer.prediction.relative_point(c.position)
            for c in self.system.plant_provider.crops
        }
        if self.current_segment:
            # Correctly filter to get upcoming crops based on their .x position
            upcoming_crop_positions = {
                c: pos for c, pos in relative_crop_positions.items()
                if self.system.field_friend.WORK_X + self.system.field_friend.DRILL_RADIUS > pos.x and pos.x <= self.current_segment.end.x + 0.02
            }
        else:
            upcoming_crop_positions = {
                c: pos for c, pos in relative_crop_positions.items()
                if self.system.field_friend.WORK_X + self.system.field_friend.DRILL_RADIUS > pos.x
            }

        # Sort the upcoming_crop_positions dictionary by the .x attribute of its values
        sorted_crops = dict(sorted(upcoming_crop_positions.items(), key=lambda item: item[1].x))

        self.crops_to_handle = sorted_crops

        relative_weed_positions = {
            w: self.system.odometer.prediction.relative_point(w.position)
            for w in self.system.plant_provider.weeds
        }
        if self.current_segment:
            # Filter to get upcoming weeds based on their .x position
            upcoming_weed_positions = {
                w: pos for w, pos in relative_weed_positions.items()
                if self.system.field_friend.WORK_X + self.system.field_friend.DRILL_RADIUS > pos.x and pos.x <= self.current_segment.end.x
            }
        else:
            upcoming_weed_positions = {
                w: pos for w, pos in relative_weed_positions.items()
                if self.system.field_friend.WORK_X + self.system.field_friend.DRILL_RADIUS > pos.x
            }

        # Sort the upcoming_weed_positions dictionary by the .x attribute of its values
        sorted_weeds = dict(sorted(upcoming_weed_positions.items(), key=lambda item: item[1].x))

        self.weeds_to_handle = sorted_weeds

    def _generate_turn_path(self, start_spline: Spline, end_spline: Spline) -> list[PathSegment]:
        splines = []
        middle_point = start_spline.end.polar(
            self.system.driver.parameters.minimum_turning_radius, start_spline.pose(1).yaw).polar(
            self.system.driver.parameters.minimum_turning_radius, start_spline.end.direction(
                end_spline.start))
        middle_pose = Pose(x=middle_point.x, y=middle_point.y,
                           yaw=start_spline.end.direction(
                               end_spline.start))
        first_turn_spline = rosys.geometry.Spline.from_poses(
            Pose(
                x=start_spline.end.x, y=start_spline.end.y,
                yaw=start_spline.pose(1).yaw),
            middle_pose)
        splines.append(first_turn_spline)
        second_turn_spline = rosys.geometry.Spline.from_poses(
            middle_pose,
            Pose(
                x=end_spline.start.x, y=end_spline.start.y,
                yaw=end_spline.pose(0).yaw))
        splines.append(second_turn_spline)
        path = [PathSegment(spline=spline) for spline in splines]
        return path

    async def _handle_plants(self) -> None:
        if self.system.field_friend.tool == 'tornado' and self.crops_to_handle:
            await self._tornado_workflow()
        # ToDo: implement workflow of other tools

    async def _tornado_workflow(self) -> None:
        self.log.info('Starting Tornado Workflow')
        while self.crops_to_handle:
            try:
                closest_crop_position = list(self.crops_to_handle.values())[0]
                if closest_crop_position.x < 0.05:
                    # do not steer while advancing on a crop
                    target = self.system.odometer.prediction.transform(Point(x=closest_crop_position.x, y=0))
                    self.log.info('target next crop')
                    await self.system.driver.drive_to(target)
                    await self.system.plant_locator.pause()
                    await self.system.puncher.punch(closest_crop_position.y, angle=self.tornado_angle)
                    await self._safe_crop_to_row(list(self.crops_to_handle.keys())[0])
                else:
                    self.log.info('follow line of crops')
                    farthest_crop = list(self.crops_to_handle.keys())[-1]
                    upcoming_world_position = self.system.odometer.prediction.transform(farthest_crop)
                    yaw = self.system.odometer.prediction.point.direction(upcoming_world_position)
                    # only apply minimal yaw corrections to avoid oversteering
                    yaw = self.system.odometer.prediction.yaw * 0.8 + yaw * 0.2
                    target = self.system.odometer.prediction.point.polar(0.03, yaw)
                    await self.system.driver.drive_to(target)
                await self._get_upcoming_crops()
                await rosys.sleep(0.5)
            except Exception as e:
                raise WorkflowException(f'Error while tornado Workflow: {e}') from e

    async def _safe_crop_to_row(self, crop: Plant) -> None:
        if self.current_row is None:
            return
        self.log.info('Saving crop to row')
        for c in self.current_row.crops:
            if c.position.distance(crop.position) < 0.02 and c.type == crop.type and c.confidence < crop.confidence:
                c.position = crop.position
                return
        if crop.confidence > 0.9:
            self.current_row.crops.append(crop)
