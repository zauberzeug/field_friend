from typing import Optional

import numpy as np
import rosys

from .safety import Safety, SafetyHardware, SafetySimulation
from .y_axis import YAxis, YAxisHardware, YAxisSimulation
from .z_axis import ZAxis, ZAxisHardware, ZAxisSimulation


class FieldFriend(rosys.hardware.Robot):
    MOTOR_GEAR_RATIO = 12.52
    WHEEL_DIAMETER = 0.23
    M_PER_TICK = WHEEL_DIAMETER * np.pi / MOTOR_GEAR_RATIO

    def __init__(
            self, *,
            wheels: rosys.hardware.Wheels,
            y_axis: Optional[YAxis] = None,
            z_axis: Optional[ZAxis] = None,
            estop: rosys.hardware.EStop,
            bms: rosys.hardware.Bms,
            safety: Safety,
            **kwargs) -> None:
        super().__init__(**kwargs)
        self.wheels = wheels
        self.y_axis = y_axis
        self.z_axis = z_axis
        self.estop = estop
        self.bms = bms
        self.safety = safety
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
                                               m_per_tick=self.M_PER_TICK,
                                               width=0.47,
                                               is_left_reversed=True)

        serial = rosys.hardware.SerialHardware(robot_brain)
        expander = rosys.hardware.ExpanderHardware(robot_brain, serial=serial)
        if with_yaxis:
            y_axis = YAxisHardware(robot_brain, expander=expander)
        else:
            y_axis = None
        if with_zaxis:
            z_axis = ZAxisHardware(robot_brain, expander=expander)
        else:
            z_axis = None
        bms = rosys.hardware.BmsHardware(robot_brain, expander=expander, rx_pin=26, tx_pin=27, num=2)
        estop = rosys.hardware.EStopHardware(robot_brain, pins={'1': 34, '2': 35})
        safety = SafetyHardware(robot_brain, estop=estop, wheels=wheels, y_axis=y_axis, z_axis=z_axis)
        modules = [can, wheels, serial, expander, y_axis, z_axis, bms, estop, safety]
        active_modules = [module for module in modules if module is not None]

        super().__init__(wheels=wheels,
                         y_axis=y_axis,
                         z_axis=z_axis,
                         estop=estop,
                         bms=bms,
                         safety=safety,
                         modules=active_modules,
                         robot_brain=robot_brain)


class FieldFriendSimulation(FieldFriend, rosys.hardware.RobotSimulation):

    def __init__(self,  with_yaxis: bool = True, with_zaxis: bool = True) -> None:
        wheels = rosys.hardware.WheelsSimulation()
        estop = rosys.hardware.EStopSimulation()
        bms = rosys.hardware.BmsSimulation()
        if with_yaxis:
            y_axis = YAxisSimulation()
        else:
            y_axis = None
        if with_zaxis:
            z_axis = ZAxisSimulation()
        else:
            z_axis = None
        safety = SafetySimulation(wheels, estop, y_axis=y_axis, z_axis=z_axis)
        super().__init__(wheels=wheels,
                         y_axis=y_axis,
                         z_axis=z_axis,
                         estop=estop,
                         bms=bms,
                         safety=safety,
                         modules=[wheels, y_axis, z_axis, bms, estop, safety])
