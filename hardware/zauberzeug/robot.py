""" import numpy as np
import rosys

from ..battery import Battery
from ..bms import BmsMessage


class Robot(rosys.hardware.Wheels):

    def __init__(self) -> None:
        super().__init__()
        self.emergency_stop: bool = False
        self.battery: Battery = Battery()

    @property
    def is_real(self) -> bool:
        return isinstance(self, RobotHardware)

    @property
    def is_simulation(self) -> bool:
        return isinstance(self, RobotSimulation)


class RobotHardware(rosys.hardware.WheelsHardware, Robot):

    def __init__(self) -> None:
        communication = rosys.hardware.SerialCommunication()
        robot_brain = rosys.hardware.RobotBrain(communication)
        super().__init__(robot_brain)
        self.robot_brain = robot_brain
        self.last_battery_request: float = 0
        self.last_can_send_error: float = 0
        self.last_battery_logging = 0

    async def on_core_output(self, words: list[str]) -> None:
        await super().on_core_output(words)
        self.emergency_stop = words.pop() == '0' or words.pop() == '0'

    async def step(self) -> None:
        await super().step()
        for time, line in await self.robot_brain.read_lines():
            if line.startswith('bms'):
                msg = BmsMessage([int(w, 16) for w in line.split()[1:]])
                msg.check()
                result = msg.interpret()
                self.battery.percentage = result.get('capacity percent')
                self.log.info(f'New step {self.battery.percentage}')
                self.battery.voltage = result.get('total voltage')
                self.battery.current = result.get('current')
                self.battery.temperature = np.mean(result['temperatures']) if 'temperatures' in result else None
                self.battery.is_charging = (self.battery.current or 0) > 0
                self.battery.last_update = rosys.time()
                if rosys.time() > self.last_battery_logging + 5 * 60:  # only log every five minutes
                    self.last_battery_logging = rosys.time()
                    self.log.info(f'battery: {self.battery.short_string}')
            if 'could not send CAN message' in line:
                self.last_can_send_error = rosys.time()
            if 'CAN timeout for msg id' in line and '(attempt 3/3)' in line:
                self.last_can_send_error = rosys.time()

        # battery requests
        battery_interval = 1.0 if rosys.time() > self.battery.last_update + 5.0 else 5.0
        if rosys.time() > self.last_battery_request + battery_interval:
            await self.robot_brain.send('bms.send(0xdd, 0xa5, 0x03, 0x00, 0xff, 0xfd, 0x77)')
            self.last_battery_request = rosys.time()

    async def stop(self) -> None:
        await self.robot_brain.send('stop()')


class RobotSimulation(rosys.hardware.WheelsSimulation, ):

    def __init__(self) -> None:
        super().__init__()

        # TODO simulate battery and emergency stop
 """
