import logging

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
        self.max_beet_search_failures: int = 10
        self.max_distance: float = 0.10

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
                self.reset_world()
                await self.catch_weeds_in_range()
                distance = await self.get_distance_to_drive()
                await self.drive_forward_to(distance)
                await rosys.sleep(0.5)
            except (DetectorError):
                self.log.exception('aborting sequence because AI was not reachable')
                return

    async def catch_weeds_in_range(self) -> None:
        self.log.info(f'punching weeds in work range')
        await self.plant_detection.check_cam(self.camera_selector.camera)
        while True:
            target_weeds = await self.get_target_weeds()
            if not target_weeds:
                self.log.info('no target weeds found')
                return
            for weed in sorted(target_weeds, key=lambda w: w.position.y, reverse=True):
                self.log.info(f' weed position which i am checking {weed.position}')
                await self.catch_weed(weed.position.y)
                self.log.info(f'punched y: {weed.position.y:.2f} with weeds {[w.position.y for w in target_weeds]}')
            await self.robot.move_yaxis_to(self.robot.MAX_Y)
            self.log.info(f'all target weeds removed')
            return

    async def get_distance_to_drive(self) -> float:
        self.reset_world()
        await self.plant_detection.check_cam(self.camera_selector.camera)
        max_distance = self.robot.AXIS_OFFSET_X + 0.05  # max distance we like to drive if no weed has been detected
        distance = max_distance
        for weed in sorted(self.plant_provider.weeds, key=lambda w: w.position.x):
            if weed.position.x > self.robot.AXIS_OFFSET_X:
                distance = weed.position.x - 0.007
                self.log.info(f'Weed in sight, at position {distance*100:.0f} cm')
                return distance
        self.log.info(f'No weed in sight, will advance max distance {max_distance*100:.0f} cm')
        return distance

    async def drive_forward_to(self, target_distance_x: float) -> None:
        min_drive_distance = 0.005
        real_distance = max(target_distance_x - self.robot.AXIS_OFFSET_X, min_drive_distance)
        target_beet = await self.get_target_beet()
        if not target_beet:
            target = rosys.geometry.Point(x=real_distance, y=0)
            self.log.info(
                f'No beet found driving straight forward, remaining beet search failure {self.max_beet_search_failures - self.beet_search_failures}')
            self.beet_search_failures += 1
        else:
            if not -0.07 < target_beet.position.y < 0.07:
                target = rosys.geometry.Point(x=real_distance, y=target_beet.position.y/2)
                self.log.info(f'found beet at y = {target.y}, driving to x={real_distance:.2f} and y={target.y:.2f}')
            else:
                target = rosys.geometry.Point(x=real_distance, y=0)
        await self.driver.drive_to(target)
        await rosys.sleep(0.5)
        await self.robot.stop()

    async def get_target_beet(self) -> Plant:
        await self.plant_detection.check_cam(self.camera_selector.camera)
        for beet in sorted(self.plant_provider.beets, key=lambda b: b.position.x):
            if beet.position.x > self.robot.AXIS_OFFSET_X:
                self.log.info(f'found beet at position {beet.position.y}')
                return beet
        self.log.info('no beet in work range')
        return None

    async def get_target_weeds(self) -> list[Plant]:
        weeds_in_range = [w for w in self.plant_provider.weeds
                          if self.robot.AXIS_OFFSET_X - 0.025 <= w.position.x <= self.robot.AXIS_OFFSET_X + 0.025 and
                          self.robot.MIN_Y <= w.position.y <= self.robot.MAX_Y]
        if not weeds_in_range:
            self.log.info(f'no weeds in work range')
            return []
        self.log.info(f'found {len(weeds_in_range)} weeds in range')
        return weeds_in_range

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
            if not await self.robot.try_reference_zaxis():
                return False
            if not await self.robot.try_reference_yaxis():
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
        # await self.robot.move_yaxis_to(self.robot.MAX_Y, speed)
        self.log.info(f'weed at {y} got catched')
        self.weed_load += 1
        await self.robot.stop()

    async def punch(self, x: float, y: float) -> None:
        if not self.robot.yaxis_is_referenced or not self.robot.zaxis_is_referenced:
            rosys.notify('axis are not referenced, homing')
            await self.start_homing()
        self.reset_world()
        await self.drive_to_punch(x)
        await self.catch_weed(y)

    async def drive_to_punch(self, target_distance_x: float) -> None:
        min_drive_distance = 0.01
        real_distance = max(target_distance_x - self.robot.AXIS_OFFSET_X, min_drive_distance)
        target = rosys.geometry.Point(x=real_distance, y=0)
        await self.driver.drive_to(target)
        await rosys.sleep(0.02)
        await self.robot.stop()

    def reset_world(self) -> None:
        self.plant_provider.clear_weeds()
        self.plant_provider.clear_beets()
        self.driver.odometer.history = []
        self.driver.odometer.prediction = rosys.geometry.Pose()
