import logging

import rosys

import hardware

from .plant import Plant
from .plant_detection import DetectorError, PlantDetection
from .plant_provider import PlantProvider


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
        self.max_weeds: int = 10
        self.crop_search_failures: int = 0
        self.max_crop_search_failures: int = 10
        self.max_distance: float = 0.10

    async def start(self) -> None:
        if self.robot.is_real:
            if not await self.robot.start_homing():
                return
        self.weed_load = 0
        self.crop_search_failures = 0
        while True:
            try:
                if self.weed_load >= self.max_weeds:
                    rosys.notify(f'stopping because weed load is {self.max_weeds}')
                    return
                if self.crop_search_failures >= self.max_crop_search_failures:
                    rosys.notify('stopping because asuming out of crop row')
                    return
                self.reset_world()
                await self.punch_weeds_in_range()
                distance = await self.get_distance_to_drive()
                await self.drive_forward_to(distance)
                await rosys.sleep(0.5)
            except (DetectorError):
                self.log.exception('aborting sequence because AI was not reachable')
                return

    async def punch_weeds_in_range(self) -> None:
        self.log.info(f'punching weeds in work range')
        await self.plant_detection.check_cam(self.camera_selector.camera)
        while True:
            target_weeds = await self.get_target_weeds()
            if not target_weeds:
                self.log.info('no target weeds found')
                return
            for weed in sorted(target_weeds, key=lambda w: w.position.y, reverse=True):
                self.log.info(f' weed position which i am checking {weed.position}')
                await self.punch_weed(weed.position.y)
                self.log.info(f'punched y: {weed.position.y:.2f} with weeds {[w.position.y for w in target_weeds]}')
            await self.robot.move_yaxis_to(self.robot.MAX_Y)
            self.log.info(f'all target weeds removed')
            return

    async def get_distance_to_drive(self) -> float:
        self.log.info(f'getting distance to drive')
        self.reset_world()
        await self.plant_detection.check_cam(self.camera_selector.camera)
        max_distance = self.robot.AXIS_OFFSET_X + 0.05  # max distance we like to drive if no weed has been detected
        distance = max_distance
        for weed in sorted(self.plant_provider.weeds, key=lambda w: w.position.x):
            if weed.position.x > self.robot.AXIS_OFFSET_X:
                distance = weed.position.x - 0.008
                self.log.info(f'Weed in sight, at position {distance*100:.0f} cm')
                return distance
        self.log.info(f'No weed in sight, will advance max distance {max_distance*100:.0f} cm')
        return distance

    async def drive_forward_to(self, target_distance_x: float) -> None:
        min_drive_distance = 0.01
        real_distance = max(target_distance_x - self.robot.AXIS_OFFSET_X, min_drive_distance)
        target_crop = await self.get_target_crop()
        if not target_crop:
            target = rosys.geometry.Point(x=real_distance, y=0)
            self.log.info(
                f'No crop found driving straight forward, remaining crop search failure {self.max_crop_search_failures - self.crop_search_failures}')
            self.crop_search_failures += 1
        else:
            if not -0.025 < target_crop.position.y < 0.025:
                target = rosys.geometry.Point(x=real_distance, y=target_crop.position.y*0.3)
                self.log.info(f'found crop, driving to x={real_distance:.2f} and y={target.y:.2f}')
            else:
                self.log.info(f'found crops in line, driving straight forward to {real_distance}')
                target = rosys.geometry.Point(x=real_distance, y=0)
        await self.driver.drive_to(target)
        await rosys.sleep(0.7)
        await self.robot.stop()

    async def get_target_crop(self) -> Plant:
        await self.plant_detection.check_cam(self.camera_selector.camera)
        for crop in sorted(self.plant_provider.crops, key=lambda b: b.position.x):
            if crop.position.x > self.robot.AXIS_OFFSET_X:
                self.log.info(f'found crop at position {crop.position.y}')
                return crop
        self.log.info('no crop in work range')
        return None

    async def get_target_weeds(self) -> list[Plant]:
        weeds_in_range = [w for w in self.plant_provider.weeds
                          if self.robot.AXIS_OFFSET_X - 0.035 <= w.position.x <= self.robot.AXIS_OFFSET_X + 0.035 and
                          self.robot.MIN_Y <= w.position.y <= self.robot.MAX_Y]
        if not weeds_in_range:
            self.log.info(f'no weeds in work range')
            return []
        self.log.info(f'found {len(weeds_in_range)} weeds in range')
        return weeds_in_range

    async def punch_weed(self, y: float) -> None:
        if not self.robot.yaxis_is_referenced or not self.robot.zaxis_is_referenced:
            rosys.notify('axis are not referenced')
            return
        speed = self.robot.AXIS_MAX_SPEED
        await self.robot.move_yaxis_to(y, speed)
        await self.robot.move_zaxis_to(self.robot.zaxis_drill_depth, speed)
        await self.robot.move_zaxis_to(self.robot.MAX_Z, speed)
        self.log.info(f'weed at {y:.3f} punched')
        self.weed_load += 1
        await self.robot.stop()

    async def punch(self, x: float, y: float, speed: float = None) -> None:
        if not self.robot.yaxis_is_referenced or not self.robot.zaxis_is_referenced:
            rosys.notify('axis are not referenced, homing..')
            await self.robot.start_homing()
        if speed == None:
            speed = self.robot.AXIS_MAX_SPEED
        self.reset_world()
        await self.drive_to_punch(x)
        await self.punch_weed(y)
        await self.robot.move_yaxis_to(self.robot.MAX_Y, speed)

    async def drive_to_punch(self, target_distance_x: float) -> None:
        min_drive_distance = 0.01
        real_distance = max(target_distance_x - self.robot.AXIS_OFFSET_X, min_drive_distance)
        target = rosys.geometry.Point(x=real_distance, y=0)
        await self.driver.drive_to(target)
        await rosys.sleep(0.02)
        await self.robot.stop()

    def reset_world(self) -> None:
        self.plant_provider.clear_weeds()
        self.plant_provider.clear_crops()
        self.driver.odometer.history = []
        self.driver.odometer.prediction = rosys.geometry.Pose()
