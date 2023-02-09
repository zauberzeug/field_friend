import rosys
from nicegui.events import KeyEventArguments

import hardware


class KeyControls(rosys.driving.keyboard_control):

    def __init__(self, robot: hardware.Robot, steerer: rosys.driving.Steerer, automator: rosys.automation.Automator) -> None:
        super().__init__(steerer)

        self.robot = robot
        self.automator = automator

    def handle_keys(self, e: KeyEventArguments) -> None:
        super().handle_keys(e)

        if e.modifiers.shift and e.action.keydown:
            if e.key == '!':
                async def try_axis_home():
                    if not await self.robot.start_homing():
                        rosys.notify('homing: failed')
                    else:
                        rosys.notify('homing successful')
                self.automator.start(try_axis_home())

        if e.modifiers.shift and e.action.keydown:
            if e.key == 'W':
                rosys.task_logger.create_task(self.robot.move_zaxis_to(self.robot.MAX_Z))
            if e.key == 'S':
                rosys.task_logger.create_task(self.robot.move_zaxis_to(self.robot.MIN_Z))
        if e.modifiers.shift and e.action.keyup:
            if e.key.name in 'WS':
                rosys.task_logger.create_task(self.robot.stop_zaxis())

        if e.modifiers.shift and e.action.keydown:
            if e.key == 'A':
                rosys.task_logger.create_task(self.robot.move_yaxis_to(self.robot.MAX_Y))
            if e.key == 'D':
                rosys.task_logger.create_task(self.robot.move_yaxis_to(self.robot.MIN_Y))
        if e.modifiers.shift and e.action.keyup:
            if e.key.name in 'AD':
                rosys.task_logger.create_task(self.robot.stop_yaxis())

        if e.key == ' ' and e.action.keydown:
            rosys.task_logger.create_task(self.robot.stop())
