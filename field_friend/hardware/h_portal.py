import abc
from typing import Optional

import rosys
from rosys.helpers import remove_indentation


class HPortal(rosys.hardware.Module, abc.ABC):
    """The y axis module is a simple example for a representation of real or simulated robot hardware."""

    MIN_X: float = 0.14865
    MAX_X: float = MIN_X + 0.25

    # Reduced by 5mm on both sides
    MIN_Y = -0.12350
    MAX_Y = MIN_Y + 0.25

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)

        self.reference_t: bool = False
        self.reference_s: bool = False
        self.left_position: int = 0
        self.right_position: int = 0
        self.left_target_reached: bool = False
        self.right_target_reached: bool = False
        self.left_fault: bool = False
        self.right_fault: bool = False
        self.previous_fault: bool = False

        self.is_referenced: bool = False

        rosys.on_shutdown(self.stop)

    @property
    def has_fault(self) -> bool:
        return self.left_fault or self.right_fault

    @property
    def both_target_reached(self) -> bool:
        return self.left_target_reached and self.right_target_reached

    @abc.abstractmethod
    async def stop(self) -> None:
        pass

    @abc.abstractmethod
    async def move_to_cartesian(self, s: float, t: float, speed: int) -> None:
        pass

    @abc.abstractmethod
    async def try_reference(self) -> bool:
        return True

    @abc.abstractmethod
    async def enable_h_motors(self) -> None:
        pass

    @abc.abstractmethod
    async def disable_h_motors(self) -> None:
        pass


class HPortalHardware(HPortal, rosys.hardware.ModuleHardware):

    def __init__(self, robot_brain: rosys.hardware.RobotBrain, *,
                 name: str = 'hportal',
                 can: rosys.hardware.CanHardware,
                 expander: Optional[rosys.hardware.ExpanderHardware],
                 left_node_id: int = 0x77,
                 right_node_id: int = 0x42,
                 ref_t_pin: int = 33,
                 ref_s_pin: int = 32,
                 sync_interval: int = 5,
                 refs_on_expander: bool = False,
                 ) -> None:
        self.name = name
        self.expander = expander
        lizard_code = remove_indentation(f'''
            master = CanOpenMaster({can.name})
            {name}_left = CanOpenMotor({can.name}, {left_node_id})
            {name}_right = CanOpenMotor({can.name}, {right_node_id})
            master.sync_interval = {sync_interval}
            {name}_ref_t = {expander.name + "." if refs_on_expander else ""}Input({ref_t_pin})
            {name}_ref_s = {expander.name + "." if refs_on_expander else ""}Input({ref_s_pin})
            bool {name}_is_referenced = false
            let {name}_brake_off do
                {name}_left.set_ctrl_halt(false)
                {name}_right.set_ctrl_halt(false)
            end
            let {name}_brake_on do
                {name}_left.set_ctrl_halt(true)
                {name}_right.set_ctrl_halt(true)
            end
            let {name}_on do
                {name}_left.set_ctrl_enable(true)
                {name}_right.set_ctrl_enable(true)
            end
            let {name}_off do
                {name}_left.set_ctrl_enable(false)
                {name}_right.set_ctrl_enable(false)
            end
            let {name}_move_to_target do
                {name}_left.commit_target_position()
                {name}_right.commit_target_position()
            end
            let {name}_reset_fault do
                {name}_left.reset_fault()
                {name}_right.reset_fault()
            end
            bool {name}_ref_run_t = false
            bool {name}_ref_run_s = false

            when {name}_ref_run_t and {name}_ref_t.level == 0 then {name}_brake_on(); end
            when {name}_ref_run_s and {name}_ref_s.level == 0 then {name}_brake_on(); end

        ''')
        core_message_fields = [
            f'{name}_ref_t.level',
            f'{name}_ref_s.level',
            f'{name}_left.actual_position',
            f'{name}_right.actual_position',
            f'{name}_left.status_target_reached',
            f'{name}_right.status_target_reached',
            f'{name}_left.status_fault',
            f'{name}_right.status_fault',
            f'{name}_is_referenced',
        ]
        super().__init__(robot_brain=robot_brain, lizard_code=lizard_code, core_message_fields=core_message_fields)

    async def stop(self) -> None:
        await super().stop()
        await self.robot_brain.send(f'{self.name}_brake_on()')

    async def enable_h_motors(self) -> None:
        await super().enable_h_motors()
        await self.robot_brain.send(f'{self.name}_on()')

    async def disable_h_motors(self) -> None:
        await super().disable_h_motors()
        await self.robot_brain.send(f'{self.name}_off()')

    async def enter_pp_mode(self, velocity: int):
        await self.robot_brain.send(
            f'{self.name}_left.enter_pp_mode({velocity});'
            f'{self.name}_right.enter_pp_mode({velocity});'
        )

    async def move_to_async(self, left: int, right: int) -> None:
        await self.robot_brain.send(
            f'{self.name}_left.set_target_position({left});'
            f'{self.name}_right.set_target_position({right});'
            f'{self.name}_move_to_target();'
        )

    async def move_to(self, left: int, right: int) -> None:
        await super().move_to()
        await self.move_to_async(left, right)

        # Give flags time to turn false first
        await rosys.sleep(0.2)
        while not self.both_target_reached:
            await rosys.sleep(0.2)

    # assumes cartesian coordinates centered on bottom left corner (center = [0.125, 0.125])
    # returns h-coordinates centered on center of h-portal
    def cartesian_m_to_h_steps(self, s: float, t: float) -> tuple:
        scaling_factor = 40000
        bottom_left_offset = 0.125
        s -= bottom_left_offset
        t -= bottom_left_offset

        right = (-s + t) * scaling_factor
        left = (-s - t) * scaling_factor

        return int(right), int(left)

    # assumes h-coordinates are centered on h-portal center (bottom left = [0, 10000])
    # returns cartesian coordinates centered on bottom left corner
    def h_steps_to_cartesian_m(self, left: int, right: int) -> tuple:
        scaling_denominator = 40000 * 2
        bottom_left_offset = 0.125
        s = (-left - right) / scaling_denominator + bottom_left_offset
        t = (left - right) / scaling_denominator + bottom_left_offset

        return s, t

    async def move_to_cartesian(self, s: float, t: float, speed: int):
        assert 0.0 <= s <= 0.25
        assert 0.0 <= t <= 0.25

        left, right = self.cartesian_m_to_h_steps(s, t)
        await self.enter_pp_mode(speed)
        await self.move_to(left, right)

    async def reference_run(self):
        # Reset offsets
        await self.robot_brain.send(
            f'{self.name}_is_referenced = true;'
            f'{self.name}_left.position_offset = 0;'
            f'{self.name}_right.position_offset = 0;'
        )

        self.log.info("enabling h motors")
        await self.enable_h_motors()

        # If already in t reference, drive out again
        if self.reference_t:
            self.log.info("already in t reference, drive out")
            await rosys.sleep(0.2)
            await self.robot_brain.send(
                f'{self.name}_ref_run_t = false;'
                f'{self.name}_ref_run_s = false;'
                f'{self.name}_left.enter_pv_mode(4);'
                f'{self.name}_right.enter_pv_mode(-4);'
                f'{self.name}_brake_off();'
            )

            while self.reference_t:
                await rosys.sleep(0.4)

        self.log.info("driving into t reference")
        await self.robot_brain.send(
            f'{self.name}_ref_run_t = true;'
            f'{self.name}_left.enter_pv_mode(-1);'
            f'{self.name}_right.enter_pv_mode(1);'
            f'{self.name}_brake_off();'
        )
        while not self.reference_t:
            await rosys.sleep(0.4)

        await self.robot_brain.send(
            f'{self.name}_ref_run_t = false;'
        )

        await rosys.sleep(0.8)
        # Assume we're roughly 110 units (both motors) away from rail end
        self.log.info("moving last step in t direction")
        T_LAST_STEP: int = 110
        left_base_local = self.left_position
        right_base_local = self.right_position
        await self.enter_pp_mode(4)
        left_base_local -= T_LAST_STEP
        right_base_local += T_LAST_STEP
        await self.move_to_async(left_base_local, right_base_local)
        await rosys.sleep(0.6)

        # If already in s reference, drive out first
        if self.reference_s:
            self.log.info("already in s reference, drive out")
            await self.robot_brain.send(
                f'{self.name}_ref_run_s = false;'
                f'{self.name}_left.enter_pv_mode(-4);'
                f'{self.name}_right.enter_pv_mode(-4);'
                f'{self.name}_brake_off()'
            )

            while self.reference_s:
                await rosys.sleep(0.4)

        self.log.info("driving into s reference")
        await self.robot_brain.send(
            f'{self.name}_ref_run_s = true;'
            f'{self.name}_left.enter_pv_mode(1);'
            f'{self.name}_right.enter_pv_mode(1);'
            f'{self.name}_brake_off()'
        )

        while not self.reference_s:
            await rosys.sleep(0.4)

        await self.robot_brain.send(
            f'{self.name}_ref_run_s = false;'
        )

        await rosys.sleep(0.8)
        left_base_local = self.left_position
        right_base_local = self.right_position

        # Drive out of both s and t reference
        self.log.info("drive out of both references")
        T_STEP_OUT = 320
        zero_point_left = left_base_local + T_STEP_OUT
        zero_point_right = right_base_local - T_STEP_OUT
        await self.enter_pp_mode(4)
        await self.move_to_async(zero_point_left, zero_point_right)
        await rosys.sleep(0.4)

        # Set offsets so left/right motor positions are now at (0, -10000)
        self.log.info("finished. set both motor offsets and referenced flag")
        await self.robot_brain.send(
            f'{self.name}_left.position_offset = {zero_point_left};'
            f'{self.name}_right.position_offset = {zero_point_right - 10000};'
        )

        await self.robot_brain.send(f'{self.name}_is_referenced = true;')
        await rosys.sleep(0.4)

    async def reset_motor_faults(self):
        await self.robot_brain.send(
            f'{self.name}_reset_fault();'
        )
        await rosys.sleep(0.4)
        self.previous_fault = False

    async def try_reference(self) -> bool:
        if not await super().try_reference():
            return False
        try:
            await self.reference_run()
            return True
        except Exception as error:
            self.log.error(f'could not reference yaxis because of {error}')
            return False

    def handle_core_output(self, time: float, words: list[str]) -> None:
        self.reference_t = words.pop(0) == '0'
        self.reference_s = words.pop(0) == '0'
        self.left_position = int(words.pop(0))
        self.right_position = int(words.pop(0))
        self.left_target_reached = (words.pop(0) == 'true')
        self.right_target_reached = (words.pop(0) == 'true')
        self.left_fault = (words.pop(0) == 'true')
        self.right_fault = (words.pop(0) == 'true')
        self.is_referenced = (words.pop(0) == 'true')

        if self.has_fault() and not self.previous_fault:
            self.previous_fault = True
            rosys.notify(f'{self.name} has fault', type='error')


class HPortalSimulation(HPortal, rosys.hardware.ModuleSimulation):

    def __init__(self) -> None:
        super().__init__()

        self.speed: int = 0
        self.target_left: Optional[float] = None
        self.target_right: Optional[float] = None

    async def stop(self) -> None:
        await super().stop()
        self.speed = 0
        self.target_left = None
        self.target_right = None

    async def move_to_cartesian(self, s: float, t: float, speed: int) -> None:
        return await super().move_to_cartesian(s, t, speed)

    async def try_reference(self) -> bool:
        return await super().try_reference()

    async def enable_h_motors(self) -> None:
        await super().enable_h_motors()
        await rosys.sleep(0.1)

    async def disable_h_motors(self) -> None:
        await super().disable_h_motors()
        await rosys.sleep(0.1)

    async def step(self, dt: float) -> None:
        await super().step(dt)
        if self.target_left is not None:
            self.left_position += int(dt * self.speed)
            if self.speed > 0:
                self.left_target_reached = self.left_position >= self.target_left
            else:
                self.left_target_reached = self.left_position <= self.target_left
            if self.left_target_reached:
                self.target_left = None
        if self.target_right is not None:
            self.right_position += int(dt * self.speed)
            if self.speed > 0:
                self.right_target_reached = self.right_position >= self.target_right
            else:
                self.right_target_reached = self.right_position <= self.target_right
            if self.right_target_reached:
                self.target_right = None
