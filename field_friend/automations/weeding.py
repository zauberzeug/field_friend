import logging
import random
from typing import Callable, Literal

import numpy as np
import rosys
from rosys.driving import Driver
from rosys.geometry import Line, Point, Point3d
from rosys.helpers import eliminate_pi, ramp
from rosys.vision import Detector

from ..hardware import FieldFriend
from ..vision import CameraSelector
from .plant import Plant
from .plant_detector import DetectorError, PlantDetector
from .plant_provider import PlantProvider
from .puncher import Puncher

CAMERA_UNCERTAINTY = 0.01
MAX_DRIVE_DISTANCE = 0.08


class Weeding:
    def __init__(self, field_friend: FieldFriend, driver: Driver, detector: Detector, camera_selector: CameraSelector,
                 plant_provider: PlantProvider, puncher: Puncher, plant_detector: PlantDetector) -> None:
        self.log = logging.getLogger('field_friend.weeding')
        self.field_friend = field_friend
        self.puncher = puncher
        self.driver = driver
        self.detector = detector
        self.plant_provider = plant_provider
        self.camera_selector = camera_selector
        self.plant_detector = plant_detector

        self.drill_depth: float = 0.05

    async def start(self) -> None:
        self.work_x = self.field_friend.WORK_X
        self.tool_radius = self.field_friend.DRILL_RADIUS
        if not await self.puncher.try_home():
            rosys.notify('Puncher homing failed, aborting', 'error')
            return
        self.plant_provider.clear()
        MAX_PUNCH_ATTEMPTS = 50
        while True:
            self.set_simulated_objects()
            for attempt in range(MAX_PUNCH_ATTEMPTS):
                try:
                    await self.punch_reachable_weeds()
                except DetectorError:
                    self.log.exception(f'Detector error (punch attempt {attempt+1}{MAX_PUNCH_ATTEMPTS})')
                    if attempt + 1 >= MAX_PUNCH_ATTEMPTS:
                        self.log.error('Too many detector errors, aborting')
                        return
                else:
                    break
            await self.drive_to_next_weed()
            await rosys.sleep(0.5)

    async def punch_reachable_weeds(self) -> None:
        self.log.info('Punching reachable weeds...')
        await self.puncher.clear_view()
        self.plant_provider.clear_weeds()
        await self.plant_detector.detect_plants(self.camera_selector.cameras['bottom cam'])
        for local_point, weed in self.sort_plants(self.plant_provider.weeds, order='y',
                                                  filter=self.field_friend.can_reach):
            self.log.info(f'Punching weed at {weed.position} and {local_point}...')
            await self.puncher.punch(local_point.y, self.drill_depth)
            self.plant_provider.remove_weed(weed)

    def sort_plants(self,
                    plants: list[Plant],
                    order: Literal['x', 'y'],
                    filter: Callable[[Point], bool] = lambda _: True,
                    reverse: bool = False,
                    ) -> list[tuple[Point, Plant]]:
        """Sort and filter plants by local x or y coordinate."""
        local_plants: list[tuple[Point, Plant]] = []
        for plant in plants:
            local_point = self.driver.odometer.prediction.relative_point(plant.position)
            if filter(local_point):
                local_plants.append((local_point, plant))
        return sorted(local_plants, key=lambda p: p[0].x if order == 'x' else p[0].y, reverse=reverse)

    async def drive_to_next_weed(self) -> None:
        self.log.info('Driving to next weed...')
        crops = self.sort_plants(self.plant_provider.crops, order='x', filter=lambda p: p.x > self.field_friend.WORK_X)
        weeds = self.sort_plants(self.plant_provider.weeds, order='x', filter=lambda p: p.x > self.field_friend.WORK_X +
                                 self.field_friend.DRILL_RADIUS and self.field_friend.y_axis.MIN_POSITION <= p.y <= self.
                                 field_friend.y_axis.MAX_POSITION)
        if crops:
            line = Line.from_points(self.driver.odometer.prediction.point, crops[0][1].position)
            self.log.info(f'Found crop, driving in direction of {crops[0][1].position}')
        else:
            line = Line.from_points(self.driver.odometer.prediction.point,
                                    self.driver.odometer.prediction.transform(Point(x=2, y=0)))

        if weeds:
            target = line.foot_point(weeds[0][1].position).polar(-self.field_friend.WORK_X, line.yaw)
            self.log.info(f'Found weed, driving to {target}')
        else:
            target = line.foot_point(self.driver.odometer.prediction.point).polar(MAX_DRIVE_DISTANCE, line.yaw)
            self.log.info(f'No weed found, driving to {target}')
        await self.drive(line, target)

    async def drive(self, line: Line, target: Point) -> None:
        hook_offset = self.driver.parameters.hook_offset
        hook_point = self.driver.odometer.prediction.transform(Point(x=hook_offset, y=0))
        carrot_distance = self.driver.parameters.carrot_distance
        carrot_tip = line.foot_point(hook_point)
        minimum_turning_radius = self.driver.parameters.minimum_turning_radius

        while True:
            while carrot_tip.distance(hook_point) < carrot_distance:
                carrot_tip = carrot_tip.polar(distance=0.01, yaw=line.yaw)

            hook_point = self.driver.odometer.prediction.transform(Point(x=hook_offset, y=0))
            turn_angle = eliminate_pi(hook_point.direction(carrot_tip) - self.driver.odometer.prediction.yaw)
            curvature = np.tan(turn_angle) / hook_offset
            if curvature != 0 and abs(1 / curvature) < minimum_turning_radius:
                curvature = (-1 if curvature < 0 else 1) / minimum_turning_radius

            target_distance = self.driver.odometer.prediction.point.projected_distance(target, line.yaw)
            linear = ramp(target_distance, hook_offset, 0.0, 1.0, 0.01, clip=True)
            angular = linear * curvature

            if target_distance > 0.01:
                await self.field_friend.wheels.drive(*self.driver._throttle(linear, angular))
                await rosys.sleep(0.1)
            else:
                await self.field_friend.wheels.stop()
                break

    def set_simulated_objects(self) -> None:
        if isinstance(self.detector, rosys.vision.DetectorSimulation):
            self.log.info('Setting simulated objects')
            self.detector.simulated_objects.clear()
            for category, x, y in [
                ('weed', random.uniform(0.05, 0.15), random.uniform(-0.12, 0.12)),
                ('weed', random.uniform(0.05, 0.15), random.uniform(-0.12, 0.12)),
                ('weed', random.uniform(0.05, 0.15), random.uniform(-0.12, 0.12)),
                ('crop', random.uniform(0.1, 0.2), random.uniform(-0.03, 0.03)),
            ]:
                obj = rosys.vision.SimulatedObject(category_name=category, position=Point3d(x=x, y=y, z=0))
                self.detector.simulated_objects.append(obj)
