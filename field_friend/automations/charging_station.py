from __future__ import annotations

import logging
from typing import TYPE_CHECKING

import rosys
from nicegui import ui
from rosys.geometry import Pose

if TYPE_CHECKING:
    from field_friend.system import System


class ChargingStation:
    DOCKING_DISTANCE = 2.0
    DOCKING_SPEED = 0.05

    def __init__(self, system: System):
        self.log = logging.getLogger(__name__)
        self.system = system
        self.mjpeg_camera_provider = system.mjpeg_camera_provider
        self.detector = system.monitoring_detector

        self.docking_distance = self.DOCKING_DISTANCE
        self.docking_speed = self.DOCKING_SPEED

        self.image_width = 1920
        self.last_detection: rosys.vision.PointDetection | None = None
        self.wip_is_charging = False

        # PID control parameters
        self.kp = 0.017  # Proportional gain (0.2/15 from original ramp)
        self.ki = 0.004  # Integral gain
        self.integral_error = 0.0
        self.last_error = 0.0
        self.dt = 0.1  # Time step

        self._detector_repeater = rosys.rosys.Repeater(self._detector_loop, 0.1)

    def _reset_pid(self):
        """Reset PID controller state"""
        self.integral_error = 0.0
        self.last_error = 0.0

    def _calculate_pid_output(self, error: float) -> float:
        """Calculate PID control output"""
        # Proportional term
        p_term = self.kp * error

        # Integral term
        self.integral_error += error * self.dt
        # Anti-windup: limit integral term
        self.integral_error = max(-0.5, min(0.5, self.integral_error))
        i_term = self.ki * self.integral_error

        # Derivative term (optional, can help with overshoot)
        d_term = 0.0  # self.kd * (error - self.last_error) / self.dt

        self.last_error = error

        return p_term + i_term + d_term

    async def approach(self):
        rosys.notify('Approaching not implemented yet')

    async def dock(self):
        rosys.notify('Docking to charging station')
        self._reset_pid()  # Reset PID state when starting

        async def move():
            self._detector_repeater.start()
            while True:
                await rosys.sleep(0.1)
                if self.wip_is_charging:
                    await self.system.field_friend.wheels.stop()
                    break
                if self.last_detection is None:
                    self.log.error('No detection')
                    await self.system.field_friend.wheels.stop()
                    continue

                # Calculate error (pixel difference from center)
                pixel_difference = self.last_detection.x - self.image_width / 2

                # Use PID controller instead of simple ramp
                angular = self._calculate_pid_output(pixel_difference)

                # Apply limits and sign
                angular = max(-0.2, min(0.2, angular))  # Limit angular velocity
                sign = 1 if angular < 0 else -1
                angular = abs(angular)

                self.log.warning(f'Pixel difference: {pixel_difference:.1f}, angular: {angular:.3f}, sign: {sign}')
                await self.system.field_friend.wheels.drive(linear=-self.docking_speed, angular=sign * angular)

            self._detector_repeater.stop()
        self.system.automator.start(move())

    async def dock_straight(self):
        rosys.notify('Docking to charging station')

        async def move():
            self._detector_repeater.start()
            robot_pose = self.system.robot_locator.pose
            with self.system.driver.parameters.set(can_drive_backwards=True, linear_speed_limit=self.docking_speed):
                await self.system.driver.drive_to(robot_pose.transform_pose(Pose(x=-self.docking_distance, y=0)).point, backward=True)
        self.system.automator.start(move())
        self._detector_repeater.stop()

    async def undock(self):
        rosys.notify('Detaching from charging station')

        async def move():
            robot_pose = self.system.robot_locator.pose
            with self.system.driver.parameters.set(linear_speed_limit=self.docking_speed):
                await self.system.driver.drive_to(robot_pose.transform_pose(Pose(x=self.docking_distance, y=0)).point)
        self.system.automator.start(move())

    async def _detector_loop(self):
        assert self.mjpeg_camera_provider is not None
        assert self.system.config is not None
        assert self.system.config.circle_sight_positions is not None
        camera_id = self.system.config.circle_sight_positions.back
        for _camera_id in self.mjpeg_camera_provider.cameras:
            if _camera_id.endswith(camera_id):
                camera_id = _camera_id
                break
        if camera_id not in self.mjpeg_camera_provider.cameras:
            self.log.error(f'Camera {camera_id} not found')
            return
        camera = self.mjpeg_camera_provider.cameras[camera_id]
        latest_image = camera.latest_captured_image
        if latest_image is None:
            self.log.error('No image for camera %s', camera_id)
            return
        self.image_width = latest_image.size.width
        assert self.detector is not None
        try:
            detections = await self.detector.detect(latest_image, tags=['charging_station', 'back'], source=self.system.robot_id)
        except rosys.vision.detector.DetectorException:
            self.log.warning('Stopped while detecting')
            return
        if detections is None:
            self.last_detection = None
            return
        for point in detections.points:
            if point.category_name == 'charging_station':
                self.last_detection = point
                self.log.debug(f'Detection: {point}')
                break

    def _toggle_detector(self, enable: bool):
        if enable:
            self._detector_repeater.start()
        else:
            self._detector_repeater.stop()

    def developer_ui(self):
        with ui.column():
            ui.label('Charging Station').classes('text-center text-bold')
            ui.number(label='Docking distance', min=0, step=0.01, format='%.3f', suffix='m', value=self.docking_distance) \
                .classes('w-4/5').bind_value_to(self, 'docking_distance')
            ui.number(label='Docking speed', min=0, step=0.01, format='%.2f', suffix='m/s', value=self.docking_speed) \
                .classes('w-4/5').bind_value_to(self, 'docking_speed')

            ui.label('PID Control').classes('text-center text-bold')
            ui.number(label='Kp (Proportional)', min=0, step=0.001, format='%.3f', value=self.kp) \
                .classes('w-4/5').bind_value_to(self, 'kp')
            ui.number(label='Ki (Integral)', min=0, step=0.0001, format='%.4f', value=self.ki) \
                .classes('w-4/5').bind_value_to(self, 'ki')
            ui.button('Reset PID', on_click=self._reset_pid)

            ui.switch('Detection', on_change=lambda e: self._toggle_detector(e.value))
            ui.button('Dock Straight', on_click=self.dock_straight)
            ui.button('Dock', on_click=self.dock)
            ui.button('Undock', on_click=self.undock)
            ui.switch('WIP is charging').bind_value_to(self, 'wip_is_charging')
            ui.label('Charging').bind_text_from(self.system.field_friend.bms.state, 'is_charging',
                                                lambda is_charging: 'Charging' if is_charging else 'Not charging')
