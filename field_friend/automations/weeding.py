import logging
import random
from functools import partial
from typing import TYPE_CHECKING, Callable, Literal, Optional

import numpy as np
import rosys
from rosys.geometry import Line, Point, Point3d, Pose, Spline
from rosys.helpers import angle, eliminate_pi, ramp

from . import DetectorError, Field, Plant, Row, find_sequence

if TYPE_CHECKING:
    from system import System

CAMERA_UNCERTAINTY = 0.02
DRILL_DRTIVE_DISTANCE = 0.04
CHOP_DRTIVE_DISTANCE = 0.10
"""max distance we like to drive if no weed has been detected"""


class Weeding:

    def __init__(self, system: 'System') -> None:
        self.log = logging.getLogger('field_friend.weeding')
        self.system = system

        self.drill_depth: float = 0.01
        self.mode: Literal['Bohren', 'Hacken', 'Beides'] = 'Hacken'
        self.plan: Optional[list[Row]] = None
        self.last_chop_position: Optional[Point] = None
        self.field: Optional[Field] = None
        self.row: Optional[Row] = None

        self.work_x: float = 0.0
        self.tool_radius: float = 0.0

    async def start(self) -> None:
        self.log.info('starting mowing')
        if self.system.field_friend.estop.active or self.system.field_friend.estop.is_soft_estop_active:
            rosys.notify('E-Stop is active', 'negative')
            return
        if self.field is None:
            rosys.notify('No field selected', 'negative')
            return
        self.log.info(f'Field: {self.field.id}')
        if self.system.gnss.device is not None:
            if self.system.gnss.device != 'simulation':
                if self.field.reference_lat is None or self.field.reference_lon is None:
                    rosys.notify('Field has no reference location set', 'negative')
                    return
                self.system.gnss.set_reference(self.field.reference_lat, self.field.reference_lon)
                distance = self.system.gnss.calculate_distance(
                    self.system.gnss.record.latitude, self.system.gnss.record.longitude)
                if not distance or distance > 1000:
                    rosys.notify(f'Distance: {distance}m to reference location is too large', 'negative')
                    return
            self.plan = self.make_plan()
        if self.system.field_friend.version in ['u2', 'u3']:
            self.work_x = self.system.field_friend.WORK_X_DRILL
            self.tool_radius = self.system.field_friend.DRILL_RADIUS
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

        self.log.info(f'mode is {self.mode}')

        self.system.plant_provider.clear()

        MAX_PUNCH_ATTEMPTS = 50
        while True:
            self.set_simulated_objects()
            for attempt in range(MAX_PUNCH_ATTEMPTS):
                try:
                    if self.mode == 'Bohren' or self.mode == 'Beides':
                        await self.punch_reachable_weeds()
                    if self.mode == 'Hacken' or self.mode == 'Beides':
                        await self.chop_weeds()
                except DetectorError:
                    self.log.exception(f'Detector error (punch attempt {attempt+1}{MAX_PUNCH_ATTEMPTS})')
                    if attempt + 1 >= MAX_PUNCH_ATTEMPTS:
                        self.log.error('Too many detector errors, aborting')
                        return
                else:
                    break
            await self.drive_forward()
            if self.needs_row_change():
                self.log.info('Changing row...')
                if len(self.plan) <= 1:
                    break
                await self.change_row()
            await rosys.sleep(0.5)

    def make_plan(self) -> Optional[list[Row]]:
        self.log.info('Making plan...')
        if not self.field.rows:
            return None

        robot_position = self.system.odometer.prediction.point
        rows = [row for row in self.field.rows if len(row.points) > 1]
        self.log.info(f'Rows: {rows}')
        if not self.row:
            row_distances = [Line.from_points(row.points[0], row.points[-1]).distance(robot_position)
                             for row in rows]
            closest_index = np.argmin(row_distances)
            self.log.info(f'Closest row: {closest_index}')
            closest_row = self.field.rows[closest_index]
        else:
            closest_row = self.row
            closest_index = self.field.rows.index(closest_row)

        sequence = find_sequence(len(rows), minimum_distance=2) or [closest_index]
        self.log.info(f'Sequence: {sequence}')
        while sequence and sequence[0] != closest_index:
            self.log.info('poppping from sequence')
            sequence.pop(0)

        closest_row_yaw = closest_row.points[0].direction(closest_row.points[-1])
        flip_first = abs(angle(closest_row_yaw, self.system.odometer.prediction.yaw)) > np.pi / 2

        plan = []
        for i, row_index in enumerate(sequence):
            row = rows[row_index]
            row.reverse = bool(flip_first == (i % 2 == 0))
            if row.reverse:
                row = row.reversed()
            plan.append(row)
        self.log.info(f'Plan: {plan}')
        return plan

    async def punch_reachable_weeds(self) -> None:
        self.log.info('Punching reachable weeds...')
        await self.system.puncher.clear_view()
        self.system.plant_provider.clear_weeds()
        async with self.system.field_friend.flashlight:
            await rosys.sleep(3.5)
            await self.system.plant_locator.detect_plants(self.system.camera_selector.cameras['bottom_cam'])
        self.keep_beets_safe()
        for local_point, weed in self.sort_plants(self.system.plant_provider.weeds, order='y', reverse=True,
                                                  filter=partial(self.system.field_friend.can_reach, second_tool=True)):
            await self.system.puncher.punch(local_point.y, self.get_drill_depth(weed.type))
            self.system.plant_provider.remove_weed(weed.id)

    def sort_plants(self,
                    plants: list[Plant],
                    order: Literal['x', 'y'],
                    filter: Callable[[Point], bool] = lambda _: True,
                    reverse: bool = False,
                    ) -> list[tuple[Point, Plant]]:
        """Sort and filter plants by local x or y coordinate."""
        local_plants: list[tuple[Point, Plant]] = []
        for plant in plants:
            local_point = self.system.odometer.prediction.relative_point(plant.position)
            if filter(local_point):
                local_plants.append((local_point, plant))
        return sorted(local_plants, key=lambda p: p[0].x if order == 'x' else p[0].y, reverse=reverse)

    def get_drill_depth(self, weed_type: str) -> float:
        """Return the drill depth for the given weed type in meters."""
        drill_depth = self.drill_depth
        if weed_type in self.system.big_weed_category_names:
            drill_depth = self.drill_depth + 0.01
        self.log.info(f'Drill depth for {weed_type}: {drill_depth}')
        return drill_depth

    async def drive_forward(self) -> None:
        self.log.info('Driving forward...')
        beets = self.sort_plants(self.system.plant_provider.crops, order='x')

        if self.plan:
            line = Line.from_points(self.plan[0].points[0], self.plan[0].points[-1])
        elif len(beets) > 10:
            self.log.info(f'beets: {beets}')
            positions = [beet[1].position for beet in beets[-10:]]
            line = Line.from_points(positions[0], positions[-1])
        else:
            line = Line.from_points(self.system.odometer.prediction.point,
                                    self.system.odometer.prediction.transform(Point(x=2, y=0)))
        self.log.info(f'driving on line {line}')

        if self.mode == 'Bohren' or self.mode == 'Beides':
            target = line.foot_point(self.system.odometer.prediction.point).polar(DRILL_DRTIVE_DISTANCE, line.yaw)
            self.log.info(f'driving to {target}')
        else:
            target = line.foot_point(self.system.odometer.prediction.point).polar(CHOP_DRTIVE_DISTANCE, line.yaw)
            self.log.info(f'driving to {target}')

        await self.drive(line, target)

    def needs_row_change(self) -> bool:
        if not self.plan:
            return False
        row_yaw = self.plan[0].points[0].direction(self.plan[0].points[-1])
        row_end_pose = Pose(x=self.plan[0].points[-1].x, y=self.plan[0].points[-1].y, yaw=row_yaw)
        return row_end_pose.relative_pose(self.system.odometer.prediction).x > 0

    async def change_row(self) -> None:
        self.log.info('Changing row...')
        spline = Spline.from_poses(
            Pose(
                x=self.plan[0].points[-1].x, y=self.plan[0].points[-1].y,
                yaw=self.plan[0].points[0].direction(self.plan[0].points[-1])),
            Pose(
                x=self.plan[1].points[0].x, y=self.plan[1].points[0].y,
                yaw=self.plan[1].points[0].direction(self.plan[1].points[-1])),)
        self.plan.pop(0)
        await self.system.driver.drive_spline(spline)

    def keep_beets_safe(self) -> None:
        """Move weeds away from beets to not harm them with tool.

        Keeps a minimum distance of TOOl_RADIUS + CAMERA_UNCERTAINTY to beets.
        Only considers plants that are close to the robot.
        """
        robot_position = self.system.odometer.prediction.point
        for beet in self.system.plant_provider.crops:
            local_beet_position = self.system.odometer.prediction.relative_point(beet.position)
            if beet.position.distance(robot_position) > 0.5:
                continue
            for weed in self.system.plant_provider.weeds:
                local_weed_position = self.system.odometer.prediction.relative_point(weed.position)
                if weed.position.distance(robot_position) > 0.5:
                    continue
                if beet.position.distance(weed.position) < self.system.field_friend.DRILL_RADIUS + CAMERA_UNCERTAINTY:
                    self.log.info(f'Beet {beet} and weed {weed} are too close, moving weed away')
                    if local_beet_position.y - local_weed_position.y > 0:
                        local_weed_position.y = local_beet_position.y - self.system.field_friend.DRILL_RADIUS - CAMERA_UNCERTAINTY
                        weed.position = self.system.odometer.prediction.transform(local_weed_position)
                    else:
                        local_weed_position.y = local_beet_position.y + self.system.field_friend.DRILL_RADIUS + CAMERA_UNCERTAINTY
                        weed.position = self.system.odometer.prediction.transform(local_weed_position)
                    self.log.info(f'Moved weed {weed} to {weed.position} to keep beets safe')
        self.system.plant_provider.PLANTS_CHANGED.emit()

    def set_simulated_objects(self) -> None:
        if isinstance(self.system.detector, rosys.vision.DetectorSimulation):
            self.log.info('Setting simulated objects')
            self.system.detector.simulated_objects.clear()
            for category, x, y in [
                ('weed', random.uniform(0.1, 0.25), random.uniform(-0.12, 0.12)),
                ('weed', random.uniform(0.1, 0.25), random.uniform(-0.12, 0.12)),
                ('weed', random.uniform(0.18, 0.25), random.uniform(-0.12, 0.12)),
                ('big_weed', random.uniform(0.1, 0.2), random.uniform(-0.12, 0.12)),
                ('big_weed', random.uniform(0.1, 0.2), random.uniform(-0.12, 0.12)),
                ('crop', 0.2, 0.01),
            ]:
                obj = rosys.vision.SimulatedObject(category_name=category, position=Point3d(x=x, y=y, z=0))
                self.system.detector.simulated_objects.append(obj)

    async def drive(self, line: Line, target: Point) -> None:
        hook_offset = self.system.driver.parameters.hook_offset
        hook_point = self.system.odometer.prediction.transform(Point(x=hook_offset, y=0))
        carrot_distance = self.system.driver.parameters.carrot_distance
        carrot_tip = line.foot_point(hook_point)
        minimum_turning_radius = self.system.driver.parameters.minimum_turning_radius

        while True:
            while carrot_tip.distance(hook_point) < carrot_distance:
                carrot_tip = carrot_tip.polar(distance=0.01, yaw=line.yaw)

            hook_point = self.system.odometer.prediction.transform(Point(x=hook_offset, y=0))
            turn_angle = eliminate_pi(hook_point.direction(carrot_tip) - self.system.odometer.prediction.yaw)
            curvature = np.tan(turn_angle) / hook_offset
            if curvature != 0 and abs(1 / curvature) < minimum_turning_radius:
                curvature = (-1 if curvature < 0 else 1) / minimum_turning_radius

            target_distance = self.system.odometer.prediction.point.projected_distance(target, line.yaw)
            linear = ramp(target_distance, hook_offset, 0.0, 1.0, 0.01, clip=True)
            angular = linear * curvature

            if target_distance > 0.01:
                await self.system.field_friend.wheels.drive(*self.system.driver._throttle(linear, angular))
                await rosys.sleep(0.1)
            else:
                await self.system.field_friend.wheels.stop()
                break

    async def chop_weeds(self) -> None:
        self.log.info('Chopping weeds...')
        await self.system.puncher.clear_view()
        self.system.plant_provider.clear_weeds()
        async with self.system.field_friend.flashlight:
            await rosys.sleep(3.5)
            await self.system.plant_locator.detect_plants(self.system.camera_selector.cameras['bottom_cam'])
        beets_in_chop_range = self.sort_plants(self.system.plant_provider.crops, order='y', reverse=True,
                                               filter=self.system.field_friend.can_reach)
        weeds_in_chop_range = self.sort_plants(self.system.plant_provider.weeds, order='y', reverse=True,
                                               filter=self.system.field_friend.can_reach)
        if not beets_in_chop_range and weeds_in_chop_range:
            self.log.info(f'weeds in chop range: {len(weeds_in_chop_range)}')
            self.log.info('No beets found in chop range, chopping weeds')
            await self.system.puncher.chop()
            for weed in weeds_in_chop_range:
                self.system.plant_provider.remove_weed(weed[1].id)
        elif not weeds_in_chop_range:
            self.log.info('No weeds found in chop range, driving forward')
        else:
            self.log.info('Beets found in chop range!, no chopping allowed')

    def estimate_line(self, *points: Point) -> Line:
        N = len(points)
        sum_y = sum(point.y for point in points)
        sum_x = sum(point.x for point in points)
        sum_yx = sum(point.y * point.x for point in points)
        sum_yy = sum(point.y ** 2 for point in points)

        m = (N * sum_yx - sum_y * sum_x) / (N * sum_yy - sum_y ** 2)
        b = (sum_x - m * sum_y) / N
        line = Line(a=-m, b=1, c=-b)
        if (points[-1].y - points[0].y) * m < 0:
            # If not, flip the line
            line = Line(a=m, b=-1, c=b)

        return line
