import logging
import math
from enum import Enum
from typing import TYPE_CHECKING, Any, Optional

import numpy as np
import rosys
from rosys.geometry import Point, Rotation
from rosys.geometry.point3d import Point3d
from rosys.vision.calibration import Calibration, Extrinsics, Intrinsics

from field_friend.vision.simulated_cam import SimulatedCam

from .plant_provider import Plant

if TYPE_CHECKING:
    from system import System

TORNADO_ANGLE = 110.0


def calc_weighted_pixel_distance(point_1: Point, point_2: Point, projection_factor: float = 1.0) -> float:
    return math.sqrt((projection_factor * (point_1.x - point_2.x))**2 + (point_1.y - point_2.y)**2)


def get_calibration(x: float, y: float, yaw: float, pitch_below_horizon: float, image_width: int = 800, image_height: int = 600):
    calibration = Calibration(
        intrinsics=Intrinsics.create_default(image_width, image_height),
        extrinsics=Extrinsics(rotation=Rotation.from_euler(np.deg2rad(
            360 - 90 - pitch_below_horizon), np.deg2rad(0), yaw + np.deg2rad(-90)), translation=[x, y, 0.4]),
    )
    return calibration


class FollowState(Enum):
    STARTUP = 0
    WAIT = 1
    TURN = 2
    DRIVE = 3


class FollowMe(rosys.persistence.PersistentModule):
    def __init__(self, system: 'System') -> None:
        super().__init__()
        self.log = logging.getLogger('field_friend.follow_me')
        self.system = system
        self.kpi_provider = system.kpi_provider

        self.stop_distance: float = 1000.0
        self.max_y_distance: float = 2000.0
        self.max_matching_distance: float = 1200.0
        self.max_first_matching_distance: float = 1200.0
        self.yaw_min: float = 0.05
        self.yaw_max: float = 0.4
        self.linear_speed: float = 0.1
        self.angular_speed: float = 0.05
        self.projection_factor: float = 0.24
        self.confidence: float = 0.1
        self.target_timeout: float = 2.0
        self.drive: bool = True

        self.state: FollowState = FollowState.STARTUP
        self.distance: float = 0.0
        self.distance_y: float = 0.0
        self.pixel_percentage_yaw: float = 0.0
        self.n_feet: int = 0
        self.target: Optional[str] = None
        self.last_update: float = rosys.time()
        self.seconds_since_update: float = 0.0

        self.pitch_below_horizon: float = 25.0
        # TODO: get resolution from camera
        self.image_width: int = 720
        self.image_height: int = 480
        self.image_center: Point = Point(x=self.image_width/2.0, y=self.image_height/2.0)

        if system.is_real:
            self.camera_id = '-3'
        else:
            self.camera_id = 'front_cam'
            self.camera = SimulatedCam(id=self.camera_id)
            self.camera.calibration = get_calibration(
                x=0.4, y=0.0, yaw=0.0, pitch_below_horizon=self.pitch_below_horizon, image_width=self.image_width, image_height=self.image_height)
            self.system.usb_camera_provider.add_camera(self.camera)
            self.image_width, self.image_height = self.camera.calibration.intrinsics.size.tuple

            self.system.plant_provider.add_weed(Plant(
                id="0",
                type='coin',
                position=Point(x=2.5, y=-0.3),
                detection_time=rosys.time(),
                confidence=0.9,
            ))
            self.system.plant_provider.add_crop(Plant(
                id="1",
                type='coin_with_hole',
                position=Point(x=5, y=2),
                detection_time=rosys.time(),
                confidence=0.9,
            ))
            self.system.plant_provider.add_crop(Plant(
                id="2",
                type='coin_with_hole',
                position=Point(x=7, y=2),
                detection_time=rosys.time(),
                confidence=0.9,
            ))
            self.system.plant_provider.add_crop(Plant(
                id="3",
                type='coin_with_hole',
                position=Point(x=10, y=-2),
                detection_time=rosys.time(),
                confidence=0.9,
            ))
            self.system.plant_provider.add_crop(Plant(
                id="4",
                type='coin_with_hole',
                position=Point(x=2, y=10),
                detection_time=rosys.time(),
                confidence=0.9,
            ))

    def backup(self) -> dict:
        return {
            'stop_distance': self.stop_distance,
            'max_y_distance': self.max_y_distance,
            'max_matching_distance': self.max_matching_distance,
            'max_first_matching_distance': self.max_first_matching_distance,
            'yaw_min': self.yaw_min,
            'yaw_max': self.yaw_max,
            'linear_speed': self.linear_speed,
            'angular_speed': self.angular_speed,
            'projection_factor': self.projection_factor,
            'confidence': self.confidence,
            'target_timeout': self.target_timeout,
            'drive': self.drive
        }

    def restore(self, data: dict[str, Any]) -> None:
        self.stop_distance = data.get('stop_distance', 1000.0)
        self.max_y_distance = data.get('max_y_distance', 2000.0)
        self.max_matching_distance = data.get('max_matching_distance', 1200.0)
        self.max_first_matching_distance = data.get('max_first_matching_distance', 1200.0)
        self.yaw_min = data.get('yaw_min', 0.1)
        self.yaw_max = data.get('yaw_max', 0.4)
        self.linear_speed = data.get('linear_speed', 0.1)
        self.angular_speed = data.get('angular_speed', 0.05)
        self.projection_factor = data.get('projection_factor', 1.0)
        self.confidence = data.get('confidence', 0.1)
        self.target_timeout = data.get('target_timeout', 2.0)
        self.drive = data.get('drive', True)

    async def start(self) -> None:
        self.log.info('starting FollowMe')

        if self.system.field_friend.estop.active or self.system.field_friend.estop.is_soft_estop_active:
            rosys.notify('E-Stop is active, aborting', 'negative')
            self.log.error('E-Stop is active, aborting')
            return
        self.state = FollowState.STARTUP
        await self.system.driver.wheels.stop()
        self.target = None
        self.system.odometer.reset()
        await self._follow_pixel()

    async def _follow_pixel(self) -> None:
        rosys.notify('FollowMe started', 'positive')
        self.log.info('FollowMe started')
        self.target = None
        if self.system.is_real:
            self.system.person_locator.is_paused = False
        try:
            while True:
                current_time = rosys.time()
                self.seconds_since_update = current_time - self.last_update
                if self.seconds_since_update > self.target_timeout or self.target is None:
                    await self.system.driver.wheels.stop()
                    self.state = FollowState.STARTUP
                    self.target = None

                image_points = await self.detect()
                if image_points is None:
                    await rosys.sleep(0.1)
                    continue
                self.n_feet = len(image_points)
                if self.n_feet == 0:
                    await rosys.sleep(0.1)
                    continue
                self.last_update = current_time

                if self.state == FollowState.STARTUP:
                    await self.system.driver.wheels.stop()
                    distances = np.array([math.sqrt((image_point.x - self.image_center.x)**2 +
                                         (image_point.y - self.image_height)**2) for image_point in image_points])
                    target_index = np.argmin(distances)
                    self.distance = distances[target_index]
                    if self.distance <= self.max_first_matching_distance:
                        self.state = FollowState.WAIT
                        self.target = image_points[target_index]
                    await rosys.sleep(0.1)
                    continue

                image_points = list(filter(lambda image_point: image_point.y > (
                    self.image_height - self.max_y_distance), image_points))
                if len(image_points) == 0:
                    self.state = FollowState.STARTUP
                    await rosys.sleep(0.1)
                    continue

                distances = np.array([calc_weighted_pixel_distance(
                    point_1=image_point, point_2=self.target, projection_factor=self.projection_factor) for image_point in image_points])
                target_index = np.argmin(distances)

                if distances[target_index] > self.max_matching_distance:
                    self.state = FollowState.STARTUP
                    await rosys.sleep(0.1)
                    continue
                self.target = image_points[target_index]

                self.pixel_percentage_yaw = (self.image_center.x - self.target.x) / self.image_center.x
                pixel_percentage_yaw_abs = abs(self.pixel_percentage_yaw)
                self.distance_y = self.image_height - self.target.y

                if self.state == FollowState.WAIT:
                    await self.system.driver.wheels.stop()
                    if pixel_percentage_yaw_abs > self.yaw_max:
                        self.state = FollowState.TURN
                        continue

                    if self.distance_y > self.stop_distance:
                        self.state = FollowState.DRIVE
                        continue

                    await rosys.sleep(0.1)
                    continue

                if self.state == FollowState.TURN:
                    if pixel_percentage_yaw_abs < self.yaw_min:
                        self.state = FollowState.DRIVE
                        continue

                    angular_speed = math.copysign(1, self.pixel_percentage_yaw) * self.angular_speed
                    # TODO: linear velocity when turning
                    # TODO: lower speed when approaching target
                    if self.drive:
                        await self.system.driver.wheels.drive(0.01, angular_speed)
                    await rosys.sleep(0.1)
                    continue

                if self.state == FollowState.DRIVE:
                    if self.distance_y < self.stop_distance:
                        await self.system.driver.wheels.stop()
                        self.state = FollowState.WAIT
                        continue

                    if pixel_percentage_yaw_abs > self.yaw_max:
                        await self.system.driver.wheels.stop()
                        self.state = FollowState.TURN
                        continue

                    # TODO: lower speed when approaching target
                    angular_speed = self.pixel_percentage_yaw if pixel_percentage_yaw_abs > self.yaw_min else 0
                    if self.drive:
                        await self.system.driver.wheels.drive(self.linear_speed, angular_speed)
                    await rosys.sleep(0.1)
                    continue
                break
        finally:
            self.request_backup()
            self.kpi_provider.increment('followme_completed')
            await rosys.sleep(0.1)
            await self.system.field_friend.stop()
            if self.system.is_real:
                self.system.person_locator.is_paused = True

    async def detect_real(self, confidence=0.0) -> Optional[list[Point]]:
        for camera in self.system.mjpeg_camera_provider.cameras.values():
            if self.camera_id in camera.id:
                self.camera = camera
                break
        image = self.camera.latest_detected_image
        if image and image.detections:
            return [p for p in image.detections.points if p.category_name == 'person' and p.confidence > confidence]
        return []

    def detect_simulated(self) -> list[Point]:
        # TODO: camera projection in scene doesn't match the detection
        self.camera = self.system.usb_camera_provider.cameras[self.camera_id]
        # two calibrations are needed because the camera in the scene is positioned relative to the robot but the extrinsics are not updated to provide the correct projection
        self.camera.calibration = get_calibration(
            x=0.4, y=0.0, yaw=0.0, pitch_below_horizon=self.pitch_below_horizon, image_width=self.image_width, image_height=self.image_height)
        camera_position_world = self.system.odometer.prediction.transform(point=Point(x=0.4, y=0))
        calibration = get_calibration(x=camera_position_world.x, y=camera_position_world.y, yaw=self.system.odometer.prediction.yaw,
                                      pitch_below_horizon=self.pitch_below_horizon, image_width=self.image_width, image_height=self.image_height)

        image_points = []
        for plant in self.system.plant_provider.crops + self.system.plant_provider.weeds:
            if plant.confidence < self.confidence:
                continue
            image_point = calibration.project_to_image(Point3d(x=plant.position.x, y=plant.position.y, z=0))
            if image_point is None:
                continue
            if image_point.x < 0 or image_point.x > self.image_width or image_point.y < 0 or image_point.y > self.image_height:
                continue
            image_points.append(image_point)
        return image_points

    async def detect(self) -> Optional[list[Point]]:
        if self.system.is_real:
            return await self.detect_real(self.confidence)
        return self.detect_simulated()
