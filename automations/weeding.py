import logging

import rosys

import hardware

from .plant import Plant, PlantProvider
from .plant_detection import DetectorError, PlantDetection


class Weeding:
    def __init__(self, robot: hardware.Robot, driver: rosys.driving.Driver, detector: rosys.vision.Detector,
                 camera_provider: rosys.vision.CameraProvider, plant_provider: PlantProvider) -> None:
        self.log = logging.getLogger('field_friend.weeding')
        self.robot = robot
        self.driver = driver
        self.detector = detector
        self.plant_provider = plant_provider
        self.camera_privder = camera_provider
        self.plant_detection = PlantDetection(self.detector, self.camera_privder, self.plant_provider, self.robot)

        self.weed_load: int = 0
        self.max_weeds: int = 5
        self.max_distance: float = 0.1

    async def start(self) -> None:
        if self.robot.is_real:
            if not await self.start_homing():
                return
        if self.weed_load >= self.max_weeds:
            rosys.notify('stopping because of max weed load')
            return
        while True:
            try:
                self.plant_provider.clear_weeds()
                self.plant_provider.clear_beets()
                target_weed = await self.get_target_weed()
                if not target_weed:
                    distance = self.max_distance + self.robot.AXIS_OFFSET_X
                    await self.drive_forward_to(distance)
                    self.log.info(f'No weed in sight, will advance {self.max_distance} cm')
                    continue
                self.log.info(
                    f'{target_weed.type}: {target_weed.id} in sight, will advance to {target_weed.position}'
                )
                await self.drive_forward_to(target_weed.position.x)
                await self.catch_weed(target_weed.position.y)
            except (DetectorError):
                self.log.exception('aborting sequence because AI was not reachable')
                return

    async def drive_forward_to(self, target_distance: float) -> None:
        distance = target_distance - self.robot.AXIS_OFFSET_X
        if distance > 0:
            start = self.driver.odometer.prediction.point.x
            target = rosys.geometry.Point(x=start+distance, y=0)
            await self.driver.drive_to(target)
        await self.robot.stop()

    async def get_target_weed(self) -> Plant:
        await self.plant_detection.check_cam()
        for weed in sorted(self.plant_provider.weeds, key=lambda w: w.position.x):
            if weed.position.x >= self.robot.AXIS_OFFSET_X and self.robot.MIN_Y <= weed.position.y <= self.robot.MAX_Y:
                return weed
            self.log.info(f'WEED FALSE WITH {weed.position.x} and {weed.position.y}')
        self.log.info('no weed is in work range')
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
        await self.robot.stop()

    async def punch(self, x: float, y: float) -> None:
        if not self.robot.yaxis_is_referenced or not self.robot.zaxis_is_referenced:
            rosys.notify('axis are not referenced')
            return
        await self.drive_forward_to(x)
        await self.catch_weed(y)
