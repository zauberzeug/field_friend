import logging

import numpy as np
import rosys

import hardware

from .plant import Plant, PlantProvider
from .plant_detection import DetectorError, PlantDetection


class Weeding:
    def __init__(self, robot: hardware.Robot, driver: rosys.driving.Driver, detector: rosys.vision.Detector,
                 camera_selector: hardware.CameraSelector, plant_provider: PlantProvider) -> None:
        self.log = logging.getLogger('field_friend.weeding')
        self.robot = robot
        self.driver = driver
        self.detector = detector
        self.plant_provider = plant_provider
        self.camera_selector = camera_selector
        self.plant_detection = PlantDetection(self.detector, self.plant_provider, self.robot)

        self.weed_load: int = 0
        self.max_weeds: int = 5
        self.beet_search_failures: int = 0
        self.max_beet_search_failures: int = 5
        self.max_distance: float = 0.1

    async def start(self) -> None:
        if self.robot.is_real:
            if not await self.start_homing():
                return
        self.weed_load = 0
        self.beet_search_failures = 0
        while True:
            try:
                if self.weed_load >= self.max_weeds:
                    rosys.notify(f'stopping because weed load is {self.max_weeds}')
                    return
                if self.beet_search_failures >= self.max_beet_search_failures:
                    rosys.notify('stopping because asuming out of beet row')
                    return
                self.plant_provider.clear_weeds()
                self.plant_provider.clear_beets()
                target_weed = await self.get_target_weed()
                if not target_weed:
                    self.log.info(f'no beet or weed found, will drive forward {self.max_distance}')
                    await self.drive_forward_to(self.max_distance + self.robot.AXIS_OFFSET_X, 0)
                    await rosys.sleep(3)
                    continue
                self.log.info(
                    f'{target_weed.type} in sight, will advance to {target_weed.position}'
                )
                await self.drive_forward_to(target_weed.position.x, 0)
                updated_target_weed = await self.get_target_weed()
                await rosys.sleep(1)
                await self.catch_weed(updated_target_weed.position.y)
                await rosys.sleep(1)
            except (DetectorError):
                self.log.exception('aborting sequence because AI was not reachable')
                return

    async def drive_forward_to(self, target_distance_x: float, target_distance_y: float) -> None:
        MAX_DISTANCE = 0.5
        distance = target_distance_x - self.robot.AXIS_OFFSET_X
        start = self.driver.odometer.prediction.point.x
        if distance > 0:
            line = self.estimate_line([b.position for b in self.plant_provider.beets])
            if line is None:
                target = rosys.geometry.Point(x=start+distance, y=0)
                linear = 0.1
                angular = 0
            else:
                target = line.foot_point(rosys.geometry.Point(x=0, y=0)).polar(
                    distance=MAX_DISTANCE, yaw=rosys.helpers.eliminate_pi(line.yaw))
                angle = self.driver.odometer.prediction.point.direction(target) * 2
                chord = self.driver.odometer.prediction.point.distance(target)
                radius = chord / (2 * np.sin(angle / 2))
                linear = self.driver.parameters.linear_speed_limit
                angular = linear / radius
                self.log.info(
                    f'driving parameters: angle:{angle} chord:{chord} radius:{radius} linear:{linear} angular:{angular}')
            while self.driver.odometer.prediction.point.x < start + distance:
                await self.robot.drive(linear, angular)
                await rosys.sleep(0.02)
            await self.robot.drive(0, 0)
        await self.robot.stop()

    async def get_target_beet(self) -> Plant:
        await self.plant_detection.check_cam(self.camera_selector.camera)
        for beet in sorted(self.plant_provider.beets, key=lambda b: b.position.x):
            if beet.position.x >= self.robot.AXIS_OFFSET_X:
                return beet
        self.log.info('no beet in work range')
        return None

    async def get_target_weed(self) -> Plant:
        await self.plant_detection.check_cam(self.camera_selector.camera)
        for weed in sorted(self.plant_provider.weeds, key=lambda w: w.position.x):
            if weed.position.x >= self.robot.AXIS_OFFSET_X - 0.01 and self.robot.MIN_Y <= weed.position.y <= self.robot.MAX_Y:
                return weed
        self.log.info('no weed in work range')
        return None

    async def start_homing(self) -> bool:
        try:
            if not self.robot.end_stops_active:
                self.log.warning('end stops not activated')
                rosys.notify('end stops not active')
                return False
            if self.robot.yaxis_end_l or self.robot.yaxis_end_r or self.robot.zaxis_end_b or self.robot.zaxis_end_t:
                self.log.warning('remove from end stops to start homing')
                rosys.notify('robot in end stops')
                return False
            if not await self.robot.try_reference_yaxis():
                return False
            if not await self.robot.try_reference_zaxis():
                return False
            return True
        finally:
            await self.robot.stop()

    async def catch_weed(self, y: float) -> None:
        if not self.robot.yaxis_is_referenced or not self.robot.zaxis_is_referenced:
            rosys.notify('axis are not referenced')
            return
        speed = self.robot.WORKING_SPEED
        await self.robot.move_yaxis_to(y, speed)
        await self.robot.move_zaxis_to(self.robot.MIN_Z, speed)
        await self.robot.move_zaxis_to(self.robot.MAX_Z, speed)
        await self.robot.move_yaxis_to(self.robot.MAX_Y, speed)
        self.log.info(f'weed at {y} got catched')
        self.weed_load += 1
        await self.robot.stop()

    async def punch(self, x: float, y: float) -> None:
        if not self.robot.yaxis_is_referenced or not self.robot.zaxis_is_referenced:
            rosys.notify('axis are not referenced')
            return
        await self.drive_forward_to(x)
        await self.catch_weed(y)

    @staticmethod
    def estimate_line(points: list[rosys.geometry.Point]) -> rosys.geometry.Line:
        if len(points) < 2:
            return None
        elif len(points) == 2:
            return rosys.geometry.Line.from_points(points[0], points[1])
        else:
            A = np.hstack(([[p.x, p.y] for p in points], np.ones((len(points), 1))))
            *_, vh = np.linalg.svd(A, full_matrices=False)
            a, b, c = -vh[-1]
            return rosys.geometry.Line(a=a, b=b, c=c)
