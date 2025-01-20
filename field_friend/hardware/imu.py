from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np
import rosys
from nicegui import ui
from rosys.geometry import Rotation

if TYPE_CHECKING:
    from rosys.hardware import WheelsSimulation


@dataclass
class ImuMeasurement:
    """Imu measurement data with corrected and uncorrected angles and angular velocities in radians."""
    time: float
    roll: float
    pitch: float
    yaw: float
    roll_corrected: float
    pitch_corrected: float
    yaw_corrected: float
    roll_velocity: float | None
    pitch_velocity: float | None
    yaw_velocity: float | None


class Imu(rosys.hardware.Imu):

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self.last_measurement: ImuMeasurement | None = None

        self.NEW_MEASUREMENT = rosys.event.Event()
        """a new measurement has been received (argument: ImuMeasurement)"""

    def emit_measurement(self) -> None:
        assert self.rotation is not None
        assert self.offset_rotation is not None
        assert self.euler is not None
        corrected_rotation = self.rotation * self.offset_rotation.T
        corrected_euler = corrected_rotation.euler
        new_measurement = ImuMeasurement(
            time=rosys.time(),
            roll=self.euler[0],
            pitch=self.euler[1],
            yaw=self.euler[2],
            roll_corrected=corrected_euler[0],
            pitch_corrected=corrected_euler[1],
            yaw_corrected=corrected_euler[2],
            roll_velocity=None,
            pitch_velocity=None,
            yaw_velocity=None,
        )
        if self.last_measurement is not None:
            d_t = rosys.time() - self.last_measurement.time
            d_roll = rosys.helpers.angle(self.last_measurement.roll, self.euler[0])
            d_pitch = rosys.helpers.angle(self.last_measurement.pitch, self.euler[1])
            d_yaw = rosys.helpers.angle(self.last_measurement.yaw, self.euler[2])

            roll_velocity = d_roll / d_t
            pitch_velocity = d_pitch / d_t
            yaw_velocity = d_yaw / d_t

            new_measurement.roll_velocity = roll_velocity
            new_measurement.pitch_velocity = pitch_velocity
            new_measurement.yaw_velocity = yaw_velocity
            self.NEW_MEASUREMENT.emit(new_measurement)
        self.last_measurement = new_measurement

    def developer_ui(self) -> None:
        ui.label('IMU').classes('text-center text-bold')
        with ui.column().classes('gap-y-1'):
            ui.label().bind_text_from(self, 'roll',
                                      lambda x: f'Roll: {np.rad2deg(x):.2f}°' if x is not None else 'Roll: N/A')
            ui.label().bind_text_from(self, 'pitch',
                                      lambda x: f'Pitch: {np.rad2deg(x):.2f}°' if x is not None else 'Pitch: N/A')
            ui.label().bind_text_from(
                self, 'yaw', lambda x: f'Yaw: {np.rad2deg(x):.2f}°' if x is not None else 'Yaw: N/A')


class ImuHardware(Imu, rosys.hardware.ImuHardware):
    ...


class ImuSimulation(Imu, rosys.hardware.ImuSimulation):
    INTERVAL = 0.1
    ROLL_NOISE = np.deg2rad(0.5)
    PITCH_NOISE = np.deg2rad(0.5)
    YAW_NOISE = np.deg2rad(1.0)

    def __init__(self, *, wheels: WheelsSimulation, interval: float = INTERVAL, roll_noise: float = ROLL_NOISE, pitch_noise: float = PITCH_NOISE, yaw_noise: float = YAW_NOISE, **kwargs) -> None:
        super().__init__(**kwargs)
        self.wheels = wheels
        self.rotation = Rotation.from_euler(0, 0, 0)
        self._roll_noise = roll_noise
        self._pitch_noise = pitch_noise
        self._yaw_noise = yaw_noise
        rosys.on_repeat(self.simulate, interval)

    def simulate(self) -> None:
        roll = np.random.normal(0, self._roll_noise)
        pitch = np.random.normal(0, self._pitch_noise)
        yaw = self.wheels.pose.yaw + np.random.normal(0, self._yaw_noise)
        self.simulate_measurement(Rotation.from_euler(roll, pitch, yaw))

    def developer_ui(self) -> None:
        super().developer_ui()
        with ui.column().classes('gap-y-1'):
            ui.number(label='Roll Noise', format='%.3f', prefix='± ', suffix='°') \
                .bind_value(self, '_roll_noise', forward=np.deg2rad, backward=np.rad2deg).classes('w-4/5')
            ui.number(label='Pitch Noise', format='%.3f', prefix='± ', suffix='°') \
                .bind_value(self, '_pitch_noise', forward=np.deg2rad, backward=np.rad2deg).classes('w-4/5')
            ui.number(label='Yaw Noise', format='%.3f', prefix='± ', suffix='°') \
                .bind_value(self, '_yaw_noise', forward=np.deg2rad, backward=np.rad2deg).classes('w-4/5')
