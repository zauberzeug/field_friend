import numpy as np
import rosys

from .e_stops import EStopHardware, EStopSimulation
from .safety import SafetyHardware, SafetySimulation
from .y_axis import YAxis, YAxisHardware, YAxisSimulation
from .z_axis import ZAxis, ZAxisHardware, ZAxisSimulation

MOTOR_GEAR_RATIO = 12.52
WHEEL_DIAMETER = 0.23
M_PER_TICK = WHEEL_DIAMETER * np.pi / MOTOR_GEAR_RATIO


class FieldFriend(rosys.hardware.Robot):
    def __init__(self, *, wheels: rosys.hardware.Wheels, y_axis: YAxis = None, z_axis: ZAxis = None, **kwargs) -> None:
        super().__init__(**kwargs)
        self.wheels = wheels
        self.y_axis = y_axis
        self.z_axis = z_axis
        rosys.on_shutdown(self.stop)

    async def stop(self) -> None:
        self.wheels.stop()
        if self.y_axis:
            await self.y_axis.stop()
        if self.z_axis:
            await self.z_axis.stop()


class FieldFriendHardware(FieldFriend, rosys.hardware.RobotHardware):

    def __init__(self, with_yaxis: bool = True, with_zaxis: bool = True) -> None:
        self.communication = rosys.hardware.SerialCommunication()
        self.robot_brain = rosys.hardware.RobotBrain(self.communication)
        if self.communication.device_path == '/dev/ttyTHS0':
            self.robot_brain.lizard_firmware.flash_params = ['xavier']
        self.can = rosys.hardware.CanHardware(self.robot_brain)
        self.wheels = rosys.hardware.WheelsHardware(self.robot_brain,
                                                    can=self.can,
                                                    left_can_address=0x000,
                                                    right_can_address=0x100,
                                                    m_per_tick=M_PER_TICK,
                                                    width=0.47,
                                                    is_right_reversed=True)
        self.estop = EStopHardware(self.robot_brain)
        self.safety = SafetyHardware(self.robot_brain)
        self.serial = rosys.hardware.SerialHardware(self.robot_brain)
        self.expander = rosys.hardware.ExpanderHardware(self.robot_brain, serial=self.serial)
        if with_yaxis:
            self.y_axis = YAxisHardware(self.robot_brain, expander=self.expander)
        if with_zaxis:
            self.z_axis = ZAxisHardware(self.robot_brain, expander=self.expander)

        super().__init__(wheels=self.wheels, y_axis=self.y_axis, z_axis=self.z_axis, modules=[
            self.can, self.wheels, self.serial, self.expander, self.y_axis, self.z_axis, self.estop, self.safety], robot_brain=self.robot_brain)


class FieldFriendSimulation(FieldFriend, rosys.hardware.RobotSimulation):

    def __init__(self,  with_yaxis: bool = True, with_zaxis: bool = True) -> None:
        self.wheels = rosys.hardware.WheelsSimulation()
        self.e_stop = EStopSimulation()
        self.safety = SafetySimulation(self.wheels, self.e_stop)
        if with_yaxis:
            self.y_axis = YAxisSimulation()
        if with_zaxis:
            self.z_axis = ZAxisSimulation()
        super().__init__(wheels=self.wheels, y_axis=self.y_axis, z_axis=self.z_axis,
                         modules=[self.wheels, self.y_axis, self.z_axis, self.e_stop, self.safety])


# class Uckerbot(FieldFriend):
#     ...


# class UckerbotHardware(Uckerbot, FieldFriendHardware):

#     def __init__(self) -> None:
#         super().__init__(with_yaxis=False)
#         self.modules.append(LidarHardware(self.robot_brain))
