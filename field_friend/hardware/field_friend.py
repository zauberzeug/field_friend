from typing import Optional

import numpy as np
import rosys

from .e_stops import EStop, EStopHardware, EStopSimulation
from .safety import SafetyHardware, SafetySimulation
from .y_axis import YAxis, YAxisHardware, YAxisSimulation
from .z_axis import ZAxis, ZAxisHardware, ZAxisSimulation

MOTOR_GEAR_RATIO = 12.52
WHEEL_DIAMETER = 0.23
M_PER_TICK = WHEEL_DIAMETER * np.pi / MOTOR_GEAR_RATIO


class FieldFriend(rosys.hardware.Robot):
    MOTOR_GEAR_RATIO = 12.52
    WHEEL_DIAMETER = 0.23
    M_PER_TICK = WHEEL_DIAMETER * np.pi / MOTOR_GEAR_RATIO

    def __init__(
            self, *,
            wheels: rosys.hardware.Wheels,
            y_axis: Optional[YAxis] = None,
            z_axis: Optional[ZAxis] = None,
            estop: EStop,
            bms: rosys.hardware.Bms,
            **kwargs) -> None:
        super().__init__(**kwargs)
        self.wheels = wheels
        self.y_axis = y_axis
        self.z_axis = z_axis
        self.estop = estop
        self.bms = bms
        rosys.on_shutdown(self.stop)

    async def stop(self) -> None:
        await self.wheels.stop()
        if self.y_axis:
            await self.y_axis.stop()
        if self.z_axis:
            await self.z_axis.stop()
        # TODO: stop other modules


class FieldFriendHardware(FieldFriend, rosys.hardware.RobotHardware):

    def __init__(self, with_yaxis: bool = True, with_zaxis: bool = True) -> None:
        communication = rosys.hardware.SerialCommunication()
        robot_brain = rosys.hardware.RobotBrain(communication)
        if communication.device_path == '/dev/ttyTHS0':
            robot_brain.lizard_firmware.flash_params = ['xavier']
        can = rosys.hardware.CanHardware(robot_brain)
        wheels = rosys.hardware.WheelsHardware(robot_brain,
                                               can=can,
                                               left_can_address=0x000,
                                               right_can_address=0x100,
                                               m_per_tick=M_PER_TICK,
                                               width=0.47,
                                               is_right_reversed=True)
        estop = EStopHardware(robot_brain)
        safety = SafetyHardware(robot_brain, estop=estop, wheels=wheels)
        serial = rosys.hardware.SerialHardware(robot_brain)
        expander = rosys.hardware.ExpanderHardware(robot_brain, serial=serial)
        if with_yaxis:
            y_axis = YAxisHardware(robot_brain, expander=expander)
        if with_zaxis:
            z_axis = ZAxisHardware(robot_brain, expander=expander)
        bms = rosys.hardware.BmsHardware(robot_brain, expander=None, rx_pin=13, tx_pin=4)

        super().__init__(wheels=wheels,
                         y_axis=y_axis,
                         z_axis=z_axis,
                         estop=estop,
                         bms=bms,
                         modules=[can, wheels, serial, expander, bms, estop, safety],
                         robot_brain=robot_brain)


class FieldFriendSimulation(FieldFriend, rosys.hardware.RobotSimulation):

    def __init__(self,  with_yaxis: bool = True, with_zaxis: bool = True) -> None:
        wheels = rosys.hardware.WheelsSimulation()
        estop = EStopSimulation()
        self.safety = SafetySimulation(wheels, estop)
        bms = rosys.hardware.BmsSimulation()
        if with_yaxis:
            y_axis = YAxisSimulation()
        if with_zaxis:
            z_axis = ZAxisSimulation()
        super().__init__(wheels=wheels,
                         y_axis=y_axis,
                         z_axis=z_axis,
                         estop=estop,
                         bms=bms,
                         modules=[wheels, y_axis, z_axis, bms, estop, self.safety])


# class Uckerbot(FieldFriend):
#     ...


# class UckerbotHardware(Uckerbot, FieldFriendHardware):

#     def __init__(self) -> None:
#         super().__init__(with_yaxis=False)
#         self.modules.append(LidarHardware(self.robot_brain))
