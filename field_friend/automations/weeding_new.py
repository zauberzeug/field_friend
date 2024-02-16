
import logging
import random
from functools import partial
from typing import TYPE_CHECKING, Callable, Literal, Optional

import numpy as np
import rosys
from rosys.driving import PathSegment
from rosys.geometry import Point, Point3d, Pose, Spline

from .field_provider import Field, Row
from .plant_locator import DetectorError
from .plant_provider import Plant
from .sequence import find_sequence

if TYPE_CHECKING:
    from system import System

CAMERA_UNCERTAINTY = 0.01
MAXIMUM_DRIVE_DISTANCE = 0.185
SAFETY_DISTANCE = 0.015
MINIMUM_BEET_CONFIDENCE = 0.8


class WeedingNew:

    def __init__(self, system: 'System') -> None:
        self.log = logging.getLogger('uckerbot.weeding')
        self.system = system
        self.system.plant_provider.ADDED_NEW_BEET.register(self._beet_encountered)
        self.system.plant_provider.ADDED_NEW_WEED.register(self._weed_encountered)
        self.new_plants_detected = False
        self.drill_depth: float = 0.11
        self.field: Optional[Field] = None
        self.start_row: Optional[Row] = None
        self.plan: Optional[list[list[PathSegment]]] = None
        self.ordered_rows: list[Row] = []
        self.current_row: Optional[Row] = None
        self.task = None

        self.running: bool = False

    async def start(self) -> None:
        self.log.info('starting weeding')
        self.system.plant_provider.clear()
        if self.system.field_friend.estop.active or self.system.field_friend.estop.is_soft_estop_active:
            rosys.notify('E-Stop is active', 'negative')
            return
        if self.system.gnss.device is None:
            rosys.notify('No GNSS device found', 'negative')
            return
        if self.field is None:
            rosys.notify('No field selected', 'negative')
            return
        if self.system.gnss.device != 'simulation':
            if self.field.reference_lat is None or self.field.reference_lon is None:
                rosys.notify('Field has no reference location set', 'negative')
                return
            self.system.gnss.set_reference(self.field.reference_lat, self.field.reference_lon)
            distance = self.system.gnss.calculate_distance(
                self.system.gnss.record.latitude, self.system.gnss.record.longitude)
            if not distance or distance > 1000:
                rosys.notify('Distance to reference location is too large', 'negative')
                return
        if self.system.field_friend.version in ['u2', 'u3']:
            if self.system.field_friend.y_axis.alarm or not self.system.field_friend.y_axis.ref_t:
                rosys.notify('Y-Axis has alarm, aborting', 'error')
                return
            if self.system.field_friend.z_axis.alarm:
                rosys.notify('Z-Axis has alarm, aborting', 'error')
                return
            if not await self.system.puncher.try_home():
                rosys.notify('Puncher homing failed, aborting', 'error')
                return
        else:
            raise NotImplementedError(self.system.field_friend.version)
        await self._weeding()

    async def _weeding(self) -> None:
        rosys.notify('Weeding started', 'positive')
        self.running = True
        while True:
            try:
                self.plan = self._make_plan()
                if not self.plan:
                    rosys.notify('No plan found', 'negative')
                    return
                await self._drive_rows()
                rosys.notify('Weeding finished', 'positive')
                return
            except DetectorError:
                self.log.error('Detector error')
                continue
            except Exception as e:
                rosys.notify(f'Weeding failed beacause of {e}', 'negative')
                self.log.exception(e)
                await self.system.field_friend.stop()
                return
            finally:
                await self.system.field_friend.stop()
                self.running = False

    def _make_plan(self) -> Optional[list[rosys.driving.PathSegment]]:
        self.log.info('Making plan...')
        if not self.field.rows:
            return

        if self.start_row is None:
            self.start_row = self.field.rows[0]
        rows = [row for row in self.field.rows if len(row.points) > 1]
        minimum_distance = 1
        if len(rows) > 1:
            rows_distance = rows[0].points[0].distance(rows[1].points[0])
            if self.system.driver.parameters.minimum_turning_radius * 2 > rows_distance:
                minimum_distance = int(
                    np.ceil(self.system.driver.parameters.minimum_turning_radius * 2 / rows_distance))

        self.log.info(f'Minimum distance: {minimum_distance}')
        if minimum_distance > 1:
            sequence = find_sequence(len(rows), minimum_distance=minimum_distance)
            if not sequence:
                self.log.warning('No sequence found')
                sequence = list(range(len(rows)))
        else:
            sequence = list(range(len(rows)))
        while sequence and self.start_row != rows[sequence[0]]:
            sequence.pop(0)

        paths = []
        for i, row_index in enumerate(sequence):
            splines = []
            row = rows[row_index]
            self.ordered_rows.append(row)
            if i % 2 != 0:
                row = row.reversed()
            row_points = row.points.copy()
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

    def _generate_turn_spline(self, start_spline: Spline, end_spline: Spline) -> list[rosys.geometry.Spline]:
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
                yaw=start_spline.start.direction(
                    start_spline.end)),
            middle_pose)
        splines.append(first_turn_spline)
        second_turn_spline = rosys.geometry.Spline.from_poses(
            middle_pose,
            Pose(
                x=end_spline.start.x, y=end_spline.start.y,
                yaw=end_spline.start.direction(
                    end_spline.end)))
        splines.append(second_turn_spline)
        path = [PathSegment(spline=spline) for spline in splines]
        return path

    async def _drive_rows(self) -> None:
        self.log.info('Driving rows...')
        await self._drive_to_start()
        for i, path in enumerate(self.plan):
            self.current_row = self.ordered_rows[i]
            for j, segment in enumerate(path):
                self.log.info(f'Driving row {i + 1}/{len(self.plan)} and segment {j + 1}/{len(path)}...')
                row_completed = False
                counter = 0  # for simulation
                await self.system.puncher.clear_view()
                await self.system.field_friend.flashlight.turn_on()
                await rosys.sleep(3)
                while not row_completed:
                    self.log.info('while not row completed...')
                    self.task = rosys.background_tasks.create(
                        self.system.driver.drive_spline(segment.spline), name='driving')
                    while not self.task.done():
                        self.log.info('while not task.done()...')
                        self.log.info(f'Counter: {counter}')
                        if self.is_simulation():
                            counter += 1
                            if counter % 2 == 0:
                                self.set_simulated_objects()
                                await rosys.sleep(0.5)
                        await self.system.plant_locator.detect_plants(self.system.camera_selector.cameras['bottom_cam'])
                        await rosys.sleep(0.1)
                        if self.new_plants_detected:
                            await self.system.field_friend.stop()
                            self.task.cancel()
                            self.new_plants_detected = False
                            rosys.notify('New plants detected')
                            await self._handle_new_plants()
                            if self.is_simulation():
                                self.system.detector.simulated_objects.clear()  # simulation
                            break
                        await rosys.sleep(0.1)

                    if self.task.cancelled():
                        self.log.info('Task cancelled')
                        continue
                    else:
                        self.log.info('Task done')
                        row_completed = True
            await self.system.field_friend.flashlight.turn_off()
            if i < len(self.plan) - 1:
                self.log.info('Turning')
                turn_splines = self._generate_turn_spline(segment.spline, self.plan[i + 1][0].spline)
                await self.system.driver.drive_path(turn_splines)
        await self.system.field_friend.stop()

    async def _drive_to_start(self) -> None:
        self.log.info('Driving to start...')
        start_pose = self.system.odometer.prediction
        end_pose = Pose(x=self.plan[0][0].spline.start.x, y=self.plan[0][0].spline.start.y,
                        yaw=self.plan[0][0].spline.start.direction(self.plan[0][0].spline.end))
        start_spline = Spline.from_poses(start_pose, end_pose)
        await self.system.driver.drive_spline(start_spline)

    def _weed_encountered(self) -> None:
        if self.new_plants_detected:
            return

        for weed in self.system.plant_provider.weeds:
            weed_relative_position = self.system.odometer.prediction.relative_point(weed.position)
            if weed_relative_position.x > self.system.field_friend.WORK_X_CHOP + self.system.field_friend.CHOP_RADIUS:
                self.new_plants_detected = True
                self.log.info('weed encountered')

    def _beet_encountered(self) -> None:
        if self.new_plants_detected:
            return
        self.new_plants_detected = True
        self.log.info('beet encountered')

    async def _handle_new_plants(self) -> None:
        self.log.info('>>>Handling new plants')
        await rosys.sleep(1)
        if not self.is_simulation():
            await self.system.plant_locator.detect_plants(self.system.camera_selector.cameras['bottom_cam'])
        if self.is_simulation() or self.system.gnss.record.gps_qual == 4:
            self._handle_row_beets()
        await self._handle_drilling()
        await self._handle_chopping()

    def _handle_row_beets(self) -> None:
        self.log.info('>>>Handling row beets')
        for beet in self.system.plant_provider.crops:
            if beet.confidence < MINIMUM_BEET_CONFIDENCE or beet.position.distance(
                    self.system.odometer.prediction.point) > 0.3:
                continue
            relative_beet_position = self.system.odometer.prediction.relative_point(beet.position)
            for row_beet in self.current_row.crops:
                row_beet_relative_position = self.system.odometer.prediction.relative_point(row_beet.position)
                if abs(relative_beet_position.x - row_beet_relative_position.x) > 0.04:
                    continue
                if beet.confidence > row_beet.confidence:
                    self.log.info('Beet already in row, replacing it')
                    self.current_row.crops.remove(row_beet)
                    self.current_row.crops.append(beet)
                else:
                    self.log.info('Beet already in row, keeping it')
                return
            self.log.info('Beet not in row, adding it')
            self.current_row.crops.append(beet)

    async def _handle_drilling(self) -> None:
        self.log.info('>>>Handling drilling')
        relative_beet_positions = [self.system.odometer.prediction.relative_point(
            beet.position) for beet in self.system.plant_provider.crops]
        for beet_position in relative_beet_positions:
            if not -0.35 < beet_position.x < 0.35 or not -0.25 < beet_position.y < 0.25:
                continue
            if beet_position.x >= self.system.field_friend.WORK_X_DRILL and self.system.field_friend.y_axis.MIN_POSITION <= beet_position.y <= self.system.field_friend.y_axis.MAX_POSITION:
                self.log.info(f'driving to beet relative position: {beet_position}')
                await self.system.driver.drive_to(self.system.odometer.prediction.transform(Point(x=(beet_position.x - self.system.field_friend.WORK_X_DRILL), y=0)))
                await self._punch_reachable_weeds()
                await rosys.sleep(1)
                break
        await self.system.puncher.clear_view()

    async def _handle_chopping(self) -> None:
        self.log.info('>>>Handling chopping')
        relative_beet_positions = [
            self.system.odometer.prediction.relative_point(beet.position) for beet in self.system.plant_provider.crops
            if beet.position.distance(self.system.odometer.prediction.point) < 0.35]
        relative_weed_positions = [
            self.system.odometer.prediction.relative_point(weed.position) for weed in self.system.plant_provider.weeds
            if weed.position.distance(self.system.odometer.prediction.point) < 0.35]
        if not relative_weed_positions:
            self.log.info('No weeds found for chopping, continue driving')
            return
        if relative_beet_positions:
            minimum_drive_distance = 0
            maximum_drive_distance = MAXIMUM_DRIVE_DISTANCE
            for i, beet_position in enumerate(relative_beet_positions):
                self.log.info(f'beet {i} relative position: {beet_position}')
                if self.system.field_friend.can_reach(beet_position):
                    beet_drive_distance = abs(
                        (self.system.field_friend.WORK_X_CHOP - self.system.field_friend.CHOP_RADIUS) - beet_position.x) + CAMERA_UNCERTAINTY
                    if beet_drive_distance > minimum_drive_distance:
                        minimum_drive_distance = beet_drive_distance
                        self.log.info(f'for beet {i} new minimum drive distance: {minimum_drive_distance}')
                if beet_position.x > self.system.field_friend.WORK_X_CHOP + self.system.field_friend.CHOP_RADIUS and self.system.field_friend.y_axis.MIN_POSITION <= beet_position.y <= self.system.field_friend.y_axis.MAX_POSITION and beet_position.x < maximum_drive_distance:
                    maximum_drive_distance = abs(
                        (self.system.field_friend.WORK_X_CHOP + self.system.field_friend.CHOP_RADIUS) - beet_position.x) - CAMERA_UNCERTAINTY
                    self.log.info(f'for beet {i} new maximum drive distance: {maximum_drive_distance}')
            for weed_position in relative_weed_positions:
                weed_drive_distance = abs(
                    (self.system.field_friend.WORK_X_CHOP - self.system.field_friend.CHOP_RADIUS) - weed_position.x)
                if weed_drive_distance < maximum_drive_distance and weed_drive_distance > minimum_drive_distance:
                    maximum_drive_distance = weed_drive_distance
                    self.log.info(f'for weed new maximum drive distance: {maximum_drive_distance}')
                else:
                    self.log.info(f'for weed no new maximum drive distance, because: {weed_drive_distance}')
            if minimum_drive_distance > maximum_drive_distance:
                self.log.info('No drive distance found')
                return
            drive_distance = max(minimum_drive_distance, maximum_drive_distance)
            if drive_distance > 0:
                self.log.info(f'Driving {drive_distance} meters')
                await self.system.driver.drive_to(self.system.odometer.prediction.transform(Point(x=(drive_distance), y=0)))
            else:
                self.log.info('No drive distance found')
        await self._chop_weeds()
        await rosys.sleep(1)

    async def _punch_reachable_weeds(self) -> None:
        self.log.info('Punching reachable weeds')
        last_punch_y = None
        self._keep_beets_safe()
        for local_point, weed in self._sort_plants(
                self.system.plant_provider.weeds, order='y', reverse=True,
                filter=partial(self.system.field_friend.can_reach, second_tool=True)):
            if last_punch_y is not None and abs(local_point.y - last_punch_y) < self.system.field_friend.DRILL_RADIUS:
                continue
            await self.system.puncher.punch(local_point.y, self._get_drill_depth(weed.type))
            self.system.plant_provider.remove_weed(weed.id)
            last_punch_y = local_point.y

    def _keep_beets_safe(self) -> None:
        """Move weeds away from beets to not harm them with tool.

        Keeps a minimum distance of TOOl_RADIUS + CAMERA_UNCERTAINTY to beets.
        Only considers plants that are close to the robot.
        """
        self.log.info('Keeping beets safe')
        for beet in self.system.plant_provider.crops:
            local_beet_position = self.system.odometer.prediction.relative_point(beet.position)
            if local_beet_position.distance(Point(x=self.system.field_friend.WORK_X_DRILL, y=0)) > 0.25:
                continue
            for weed in self.system.plant_provider.weeds:
                local_weed_position = self.system.odometer.prediction.relative_point(weed.position)
                if local_weed_position.distance(local_beet_position) > 0.2:
                    continue
                if beet.position.distance(
                        weed.position) < self.system.field_friend.DRILL_RADIUS + CAMERA_UNCERTAINTY + SAFETY_DISTANCE:
                    self.log.info(f'Beet and weed are too close, moving weed {local_weed_position} away')
                    if (local_beet_position.y - local_weed_position.y) > 0:
                        local_weed_position.y = local_beet_position.y - self.system.field_friend.DRILL_RADIUS - CAMERA_UNCERTAINTY - SAFETY_DISTANCE
                        weed.position = self.system.odometer.prediction.transform(local_weed_position)
                    else:
                        local_weed_position.y = local_beet_position.y + self.system.field_friend.DRILL_RADIUS + CAMERA_UNCERTAINTY
                        weed.position = self.system.odometer.prediction.transform(local_weed_position)
                        self.log.info(
                            f'Moved weed to {local_weed_position} to keep beets safe')
        self.system.plant_provider.PLANTS_CHANGED.emit()

    def _sort_plants(self,
                     plants: list[Plant],
                     order: Literal['x', 'y'],
                     filter: Callable[[Point], bool] = lambda _: True,
                     reverse: bool = False,
                     ) -> list[tuple[Point, Plant]]:
        """Sort and filter plants by local x or y coordinate."""
        self.log.info(f'Sorting plants by {order}')
        local_plants: list[tuple[Point, Plant]] = []
        for plant in plants:
            local_point = self.system.odometer.prediction.relative_point(plant.position)
            if filter(local_point):
                local_plants.append((local_point, plant))
        return sorted(local_plants, key=lambda p: p[0].x if order == 'x' else p[0].y, reverse=reverse)

    def _get_drill_depth(self, weed_type: str) -> float:
        """Return the drill depth for the given weed type in meters."""
        drill_depth = self.drill_depth
        if weed_type in self.system.big_weed_category_names:
            drill_depth = self.drill_depth + 0.01
        self.log.info(f'Drill depth for {weed_type}: {drill_depth}')
        return drill_depth

    async def _chop_weeds(self) -> None:
        self.log.info('Chopping weeds')
        beets_in_chop_range = self._sort_plants(self.system.plant_provider.crops, order='y', reverse=True,
                                                filter=self.system.field_friend.can_reach)
        weeds_in_chop_range = self._sort_plants(self.system.plant_provider.weeds, order='y', reverse=True,
                                                filter=self.system.field_friend.can_reach)
        if not beets_in_chop_range and weeds_in_chop_range:
            self.log.info(f'weeds in chop range: {len(weeds_in_chop_range)}')
            self.log.info('No beets found in chop range, chopping weeds')
            await self.system.puncher.chop()
            for weed in weeds_in_chop_range:
                self.system.plant_provider.remove_weed(weed[1].id)

        else:
            self.log.info('Beets found in chop range!, no chopping allowed')

    def set_simulated_objects(self) -> None:
        if isinstance(self.system.detector, rosys.vision.DetectorSimulation):
            self.log.info('Setting simulated objects')
            self.system.detector.simulated_objects.clear()
            for category, x, y in [
                ('weed', 0.14, random.uniform(-0.12, 0.12)),
                ('weed', 0.21, random.uniform(-0.12, 0.12)),
                ('big_weed', 0.14, random.uniform(-0.12, 0.12)),
                ('big_weed', 0.23, random.uniform(-0.12, 0.12)),
                ('big_weed', 0.26, random.uniform(-0.12, 0.12)),
                ('crop', 0.25, 0.01),
            ]:
                obj = rosys.vision.SimulatedObject(category_name=category, position=Point3d(x=x, y=y, z=0))
                self.system.detector.simulated_objects.append(obj)

    def is_simulation(self) -> bool:
        return isinstance(self.system.detector, rosys.vision.DetectorSimulation)
