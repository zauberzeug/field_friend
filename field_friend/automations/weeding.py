import logging

import rosys
from rosys.driving import Driver
from rosys.geometry import Point, Pose
from rosys.vision import Detector

from ..hardware import FieldFriend, FieldFriendSimulation
from ..vision import CameraSelector
from .plant import Plant
from .plant_detection import DetectorError, PlantDetection
from .plant_provider import PlantProvider
from .puncher import Puncher


class Weeding:
    def __init__(self, field_friend: FieldFriend, driver: Driver, detector: Detector,
                 camera_selector: CameraSelector, plant_provider: PlantProvider, puncher: Puncher) -> None:
        self.log = logging.getLogger('field_friend.weeding')
        self.field_friend = field_friend
        self.puncher = puncher
        self.driver = driver
        self.detector = detector
        self.plant_provider = plant_provider
        self.camera_selector = camera_selector
        self.plant_detection = PlantDetection(self.detector, self.plant_provider, self.field_friend)

        self.weed_load: int = 0
        self.max_weeds: int = 10
        self.crop_search_failures: int = 0
        self.max_crop_search_failures: int = 10
        self.max_distance: float = 0.10

    def is_simulation(self) -> bool:
        return isinstance(self.field_friend, FieldFriendSimulation)

    async def start(self) -> None:
        await self.puncher.home()
        await self.reset_world()
        self.weed_load = 0
        self.crop_search_failures = 0
        while True:
            try:
                if self.is_simulation:
                    self.plant_detection.place_simulated_objects()
                if self.weed_load >= self.max_weeds:
                    rosys.notify(f'stopping because weed load is {self.max_weeds}')
                    return
                if self.crop_search_failures >= self.max_crop_search_failures:
                    rosys.notify('stopping because asuming out of crop row')
                    return
                await self.reset_world()
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
                await self.puncher.punch(weed.position.y)
                self.log.info(f'punched y: {weed.position.y:.2f} with weeds {[w.position.y for w in target_weeds]}')
            await self.field_friend.y_axis.move_to(self.field_friend.y_axis.MAX_Y)
            self.log.info(f'all target weeds removed')
            return

    async def get_distance_to_drive(self) -> float:
        self.log.info(f'getting distance to drive')
        await self.reset_world()
        await self.plant_detection.check_cam(self.camera_selector.camera)
        # max distance we like to drive if no weed has been detected
        max_distance = self.field_friend.y_axis.AXIS_OFFSET_X + 0.05
        distance = max_distance
        for weed in sorted(self.plant_provider.weeds, key=lambda w: w.position.x):
            if weed.position.x > self.field_friend.y_axis.AXIS_OFFSET_X:
                distance = weed.position.x - 0.008
                self.log.info(f'Weed in sight, at position {distance*100:.0f} cm')
                return distance
        self.log.info(f'No weed in sight, will advance max distance {max_distance*100:.0f} cm')
        return distance

    async def drive_forward_to(self, target_distance_x: float) -> None:
        min_drive_distance = 0.01
        real_distance = max(target_distance_x - self.field_friend.y_axis.AXIS_OFFSET_X, min_drive_distance)
        target_crop = await self.get_target_crop()
        if not target_crop:
            target = Point(x=real_distance, y=0)
            self.log.info(
                f'no crop found driving straight forward, remaining crop search failure {self.max_crop_search_failures - self.crop_search_failures}')
            self.crop_search_failures += 1
        else:
            if not -0.025 < target_crop.position.y < 0.025:
                target = Point(x=real_distance, y=target_crop.position.y*0.3)
                self.log.info(f'found crop, driving to x={real_distance:.2f} and y={target.y:.2f}')
            else:
                self.log.info(f'found crops in line, driving straight forward to {real_distance}')
                target = Point(x=real_distance, y=0)
        await self.driver.drive_to(target)
        await rosys.sleep(0.7)
        await self.field_friend.stop()

    async def get_target_crop(self) -> Plant:
        await self.plant_detection.check_cam(self.camera_selector.camera)
        for crop in sorted(self.plant_provider.crops, key=lambda b: b.position.x):
            if crop.position.x > self.field_friend.y_axis.AXIS_OFFSET_X:
                self.log.info(f'found crop at position {crop.position.y}')
                return crop
        self.log.info('no crop in work range')
        return None

    async def get_target_weeds(self) -> list[Plant]:
        weeds_in_range = [
            w for w in self.plant_provider.weeds
            if self.field_friend.y_axis.AXIS_OFFSET_X - 0.035 <= w.position.x <= self.field_friend.y_axis.AXIS_OFFSET_X + 0.035 and
            self.field_friend.y_axis.MIN_Y <= w.position.y <= self.field_friend.y_axis.MAX_Y]
        if not weeds_in_range:
            self.log.info(f'no weeds in work range')
            return []
        self.log.info(f'found {len(weeds_in_range)} weeds in range')
        return weeds_in_range

    async def reset_world(self) -> None:
        self.log.info('resetting world')
        self.plant_provider.clear_weeds()
        self.plant_provider.clear_crops()
        self.driver.odometer.history = []
        self.driver.odometer.prediction = Pose()
