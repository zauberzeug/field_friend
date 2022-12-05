from typing import Optional

import numpy as np
import rosys
from rosys.geometry import Velocity
from rosys.helpers import ramp

from .robot import Robot


class RobotSimulation(Robot):

    def __init__(self) -> None:
        super().__init__()

        self.pose: rosys.geometry.Pose = rosys.geometry.Pose()
        self.linear_velocity: float = 0
        self.angular_velocity: float = 0
        self.is_charging: bool = False
        self.last_update: Optional[float] = None
        self.yaxis_velocity: float = 0.0
        self.zaxis_velocity: float = 0.0
        self.yaxis_target: Optional[float] = None
        self.zaxis_target: Optional[float] = None

    async def drive(self, linear: float, angular: float) -> None:
        if self.emergency_stop:
            return
        await super().drive(linear, angular)
        self.linear_velocity = self.linear_target_speed
        self.angular_velocity = self.angular_target_speed

    async def stop_yaxis(self) -> None:
        await super().stop_yaxis()
        self.yaxis_velocity = 0
        self.yaxis_target = None

    async def stop_zaxis(self) -> None:
        await super().stop_zaxis()
        self.zaxis_velocity = 0
        self.zaxis_target = None

    async def try_reference_yaxis(self) -> None:
        if not await super().try_reference_yaxis():
            return False
        self.yaxis_position = 0
        self.yaxis_is_referenced = True
        return True

    async def try_reference_zaxis(self) -> None:
        if not await super().try_reference_zaxis():
            return False
        self.zaxis_position = 0
        self.zaxis_is_referenced = True
        return True

    async def move_yaxis_to(self, y_world_position: float, speed: float = 80000) -> None:
        await super().move_yaxis_to()
        if not self.yaxis_is_referenced:
            self.log.info('yaxis ist not referenced')
            rosys.notify('yaxis ist not referenced')
            return
        if self.yaxis_end_l or self.yaxis_end_r or self.emergency_stop:
            self.log.info('yaxis is in end stops')
            rosys.notify('yaxis is in end stops')
            return
        assert speed <= self.WORKING_SPEED
        assert self.MIN_Y <= y_world_position <= self.MAX_Y
        y_axis_position: float = y_world_position - self.AXIS_OFFSET_Y
        steps = self.linear_to_steps(y_axis_position)
        self.yaxis_target = self.yaxis_home_position + steps
        if self.yaxis_target > self.yaxis_position:
            self.yaxis_velocity = speed
        if self.yaxis_target < self.yaxis_position:
            self.yaxis_velocity = -speed

    async def move_zaxis_to(self, z_world_position: float, speed: float = 80000) -> None:
        await super().move_zaxis_to()
        if not self.zaxis_is_referenced:
            self.log.info('zaxis ist not referenced')
            rosys.notify('zaxis ist not referenced')
            return
        if self.zaxis_end_t or self.zaxis_end_b or self.emergency_stop:
            self.log.info('zaxis is in end stops')
            rosys.notify('zaxis is in end stops')
            return
        assert speed <= self.WORKING_SPEED
        assert self.MIN_Z <= z_world_position <= self.MAX_Z
        steps = self.depth_to_steps(z_world_position)
        self.zaxis_target = self.zaxis_home_position + steps
        if self.zaxis_target > self.zaxis_position:
            self.zaxis_velocity = speed
        if self.zaxis_target < self.zaxis_position:
            self.zaxis_velocity = -speed

    async def update(self) -> None:
        await super().update()

        # time
        dt = rosys.time() - self.last_update if self.last_update is not None else 0
        self.last_update = rosys.time()

        # odometry
        self.pose += rosys.geometry.PoseStep(linear=dt*self.linear_velocity,
                                             angular=dt*self.angular_velocity, time=rosys.time())
        velocity = Velocity(linear=self.linear_velocity, angular=self.angular_velocity, time=rosys.time())
        self.VELOCITY_MEASURED.emit([velocity])

        # yaxis
        self.yaxis_position += int(dt * self.yaxis_velocity)
        if self.yaxis_target is not None:
            if (self.yaxis_velocity > 0) == (self.yaxis_position > self.yaxis_target):
                self.yaxis_position = self.yaxis_target
                self.yaxis_target = None
                self.yaxis_velocity = 0

        # zaxis
        self.zaxis_position += int(dt * self.zaxis_velocity)
        if self.zaxis_target is not None:
            if (self.zaxis_velocity > 0) == (self.zaxis_position > self.zaxis_target):
                self.zaxis_position = self.zaxis_target
                self.zaxis_target = None
                self.zaxis_velocity = 0

        # battery
        self.battery.is_charging = self.is_charging
        self.battery.voltage = 25.0 + np.sin(0.01 * rosys.time())
        self.battery.percentage = ramp(self.battery.voltage, 24.0, 26.0, 30.0, 70.0)
        self.battery.current = 1.0 if self.battery.is_charging else -0.7
        self.battery.temperature = 20.0 + np.sin(0.01 * rosys.time())
        self.battery.last_update = rosys.time()
