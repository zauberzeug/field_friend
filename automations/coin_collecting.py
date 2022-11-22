import logging

import rosys

import hardware

from .coin import Coin, CoinProvider
from .coin_detection import CoinDetection, DetectorError


class CoinCollecting:
    def __init__(self, robot: hardware.Robot, driver: rosys.driving.Driver, detector: rosys.vision.Detector,
                 camera_provider: rosys.vision.CameraProvider, coin_provider: CoinProvider) -> None:
        self.log = logging.getLogger('field_friend.coin_collecting')
        self.robot = robot
        self.driver = driver
        self.detector = detector
        self.coin_provider = coin_provider
        self.camera_privder = camera_provider
        self.coin_detection = CoinDetection(self.detector, self.camera_privder, self.coin_provider, self.robot)

        self.coin_load: int = 0
        self.max_coins: int = 5
        self.max_distance: float = 0.1

    async def start(self) -> None:
        if self.robot.is_real():
            if not await self.start_homing():
                return
        if self.coin_load >= self.max_coins:
            rosys.notify('stopping because of max coin load')
            return
        while True:
            try:
                await self.coin_provider.clear()
                await self.coin_detection.update_coins()
                target_coin = await self.get_target_coin()
                if not target_coin:
                    distance = self.max_distance + self.robot.AXIS_OFFSET_X
                    await self.drive_forward_to(distance)
                    self.log.info(f'No coin in sight, will advance {self.max_distance} cm')
                else:
                    await self.drive_forward_to(target_coin.position.x)
                    await self.catch_coin(target_coin.position.y)
            except (DetectorError):
                self.log.exception('aborting sequence because AI was not reachable')

    async def drive_forward_to(self, target_distance: float) -> None:
        distance = target_distance - self.robot.AXIS_OFFSET_X
        if distance >= 0:
            start = self.driver.odometer.prediction.point.x
            while self.driver.odometer.prediction.point.x < start + distance:
                await self.robot.drive(0.2, 0)
                await rosys.sleep(0.02)
                self.log.info(f'odometer: {self.driver.odometer.prediction.point.x}')
        if distance < 0:
            start = self.driver.odometer.prediction.point.x
            while self.driver.odometer.prediction.point.x > start + distance:
                await self.robot.drive(-0.2, 0)
                await rosys.sleep(0.02)
        await self.robot.stop()
        await rosys.sleep(0.4)

    async def get_target_coin(self) -> Coin:
        await self.coin_detection.check_cam()
        for coin in sorted(self.coin_provider.coin, key=lambda c: c.position.x):
            return coin
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

    async def catch_coin(self, y: float) -> None:
        if not self.robot.yaxis_is_referenced or not self.robot.zaxis_is_referenced:
            rosys.notify('axis are not referenced')
            return
        speed = self.robot.WORKING_SPEED
        await self.robot.move_yaxis_to(y, speed)
        await self.robot.move_zaxis_to(self.robot.MIN_Z, speed)
        await self.robot.move_zaxis_to(self.robot.MAX_Z, speed)
        await self.robot.move_yaxis_to(self.robot.MAX_Y, speed)
        self.log.info(f'coin at {y} got catched')
        await self.robot.stop()
