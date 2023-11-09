import abc
from typing import Optional

import rosys
from rosys.helpers import remove_indentation


class Tornado(rosys.hardware.Module, abc.ABC):

    def __init__(self, min_position, **kwargs) -> None:
        super().__init__(**kwargs)
        self.min_position: float = min_position

        self.position_z: float = 0.0
        self.position_turn: float = 0.0

        self.is_referenced: bool = False

        self.end_top: bool = False
        self.end_bottom: bool = False

        self.ref_motor: bool = False
        self.ref_gear: bool = False
        self.ref_t: bool = False
        self.ref_b: bool = False

        rosys.on_shutdown(self.stop)

    @abc.abstractmethod
    async def stop_z(self) -> None:
        pass

    @abc.abstractmethod
    async def stop_turn(self) -> None:
        pass

    async def stop(self) -> None:
        await self.stop_z()
        await self.stop_turn()

    @abc.abstractmethod
    async def move_to(self, position: float) -> None:
        if not self.is_referenced:
            raise RuntimeError('zaxis is not referenced, reference first')
        if not self.min_position <= position <= self.min_position:
            raise RuntimeError('zaxis depth is out of range')

    @abc.abstractmethod
    async def move_down_until_reference(self) -> None:
        if not self.is_referenced:
            raise RuntimeError('yaxis is not referenced, reference first')

    @abc.abstractmethod
    async def turn_by(self, angle: float) -> None:
        if not self.is_referenced:
            raise RuntimeError('zaxis is not referenced, reference first')

    @abc.abstractmethod
    async def try_reference_z(self) -> bool:
        return True

    @abc.abstractmethod
    async def try_reference_turn(self) -> bool:
        return True

    async def try_reference(self) -> bool:
        if not await self.try_reference_z():
            return False
        if not await self.try_reference_turn():
            return False
        self.is_referenced = True
        return True

    async def return_to_reference(self) -> bool:
        try:
            await self.move_to(0)
        except RuntimeError as e:
            rosys.notify(e, type='negative')
            return False
        return True


class TornadoHardware(Tornado, rosys.hardware.ModuleHardware):
    """The z axis module is a simple example for a representation of real or simulated robot hardware."""

    def __init__(self, robot_brain: rosys.hardware.RobotBrain, *,
                 name: str = 'tornado',
                 can: rosys.hardware.CanHardware,
                 expander: Optional[rosys.hardware.ExpanderHardware],
                 min_position,
                 z_can_address: int = 0x400,
                 turn_can_address: int = 0x500,
                 m_per_tick: float = 0.01,
                 end_top_pin: int = 32,
                 end_bottom_pin: int = 5,
                 ref_motor_pin: int = 33,
                 ref_gear_pin: int = 4,
                 ref_t_pin: int = 35,
                 ref_b_pin: int = 18,
                 motors_on_expander: bool = False,
                 end_stops_on_expander: bool = True,
                 is_z_reversed: bool = False,
                 is_turn_reversed: bool = False,
                 speed_limit: int = 1,
                 current_limit: int = 20,
                 ) -> None:
        self.name = name
        self.expander = expander
        self.speed_limit = speed_limit
        self.current_limit = current_limit

        lizard_code = remove_indentation(f'''
            {name}_z = {expander.name + "." if motors_on_expander and expander else ""}ODriveMotor({can.name}, {z_can_address})
            {name}_turn = {expander.name + "." if motors_on_expander and expander else ""}ODriveMotor({can.name}, {turn_can_address})
            {name}_z.m_per_tick = {m_per_tick}
            {name}_turn.m_per_tick = {1/12.52} 
            {name}_z.limits({self.speed_limit}, {self.current_limit})
            {name}_turn.limits({self.speed_limit}, {self.current_limit})
            {name}_z.reversed = {'true' if is_z_reversed else 'false'}
            {name}_turn.reversed = {'true' if is_turn_reversed else 'false'}

            {name}_end_top = {expander.name + "." if end_stops_on_expander and expander else ""}Input({end_top_pin})
            {name}_end_bottom = {expander.name + "." if end_stops_on_expander and expander else ""}Input({end_bottom_pin})
            {name}_ref_motor = {expander.name + "." if end_stops_on_expander and expander else ""}Input({ref_motor_pin})
            {name}_ref_gear = {expander.name + "." if end_stops_on_expander and expander else ""}Input({ref_gear_pin})
            {name}_ref_t = {expander.name + "." if end_stops_on_expander and expander else ""}Input({ref_t_pin})
            {name}_ref_b = {expander.name + "." if end_stops_on_expander and expander else ""}Input({ref_b_pin})

            bool {name}_z_is_referencing = false;
            bool {name}_end_top_enabled = true;
            bool {name}_end_bottom_enabled = true;
            when {name}_end_top_enabled and {name}_z_is_referencing and {name}_end_top.level == 0 then
                {name}_z.speed(0);
                {name}_end_top_enabled = false;
            end
            when !{name}_end_top_enabled and {name}_z_is_referencing and {name}_end_top.level == 1 then 
                {name}_z.speed(0); 
                {name}_end_top_enabled = true;
            end
            when {name}_end_bottom_enabled and {name}_end_bottom.level == 0 then
                {name}_z.speed(0);
            end 

            bool {name}_turn_is_referencing = false;
            bool {name}_ref_motor_enabled = false;
            bool {name}_ref_gear_enabled = false;
            when {name}_ref_motor_enabled and {name}_turn_is_referencing and {name}_ref_motor.level == 0 then
                {name}_turn.speed(0);
                {name}_ref_motor_enabled = false;
            end

            when {name}_ref_gear_enabled and {name}_turn_is_referencing and {name}_ref_gear.level == 1 then
                {name}_turn.speed(0);
                {name}_ref_gear_enabled = false;
            end

            bool {name}_ref_t_enabled = false;
            bool {name}_ref_b_enabled = false;

            when {name}_ref_t_enabled and {name}_ref_t.level == 0 then
                {name}_z.speed(0);
                {name}_ref_t_enabled = false;
            end

            when {name}_ref_b_enabled and {name}_ref_b.level == 1 then
                {name}_z.speed(0);
                {name}_ref_b_enabled = false;
            end


        ''')
        core_message_fields = [
            f'{name}_end_top.level',
            f'{name}_end_bottom.level',
            f'{name}_ref_motor.level',
            f'{name}_ref_gear.level',
            f'{name}_ref_t.level',
            f'{name}_ref_b.level',
            f'{name}_z.position',
            f'{name}_turn.position',
        ]
        super().__init__(
            min_position=min_position,
            robot_brain=robot_brain,
            lizard_code=lizard_code,
            core_message_fields=core_message_fields
        )

    async def stop_z(self) -> None:
        await super().stop_z()
        await self.robot_brain.send(f'{self.name}_z.speed(0)')

    async def stop_turn(self) -> None:
        await super().stop_turn()
        await self.robot_brain.send(f'{self.name}_turn.speed(0)')

    async def move_to(self, position: float) -> None:
        try:
            await super().move_to(position)
        except RuntimeError as e:
            raise Exception(e)
        await self.robot_brain.send(f'{self.name}_z.position({position});')

    async def move_down_until_reference(self) -> None:
        try:
            await super().move_down_until_reference()
        except RuntimeError as e:
            raise Exception(e)
        await self.robot_brain.send(
            f'{self.name}_ref_b_enabled = true;'
            f'{self.name}_ref_t_enabled = true;'
            f'{self.name}_z.speed({-self.speed_limit});')
        while self.ref_b and not self.ref_t and self.position_z > self.min_position:
            await rosys.sleep(0.2)
        await self.robot_brain.send(
            f'{self.name}_z.speed(0);'
            f'{self.name}_ref_b_enabled = false;'
            f'{self.name}_ref_t_enabled = false;'
        )
        if self.ref_t:
            raise Exception('Error while moving z axis down: Ref Top is active')
        if self.position_z <= self.min_position:
            raise Exception('Error while moving z axis down: Position is below min position')

    async def turn_by(self, angle: float) -> None:
        try:
            await super().turn_by(angle)
        except RuntimeError as e:
            raise Exception(e)
        await self.robot_brain.send(f'{self.name}_turn.position({angle/360});')

    async def try_reference_z(self) -> bool:
        if not await super().try_reference_z():
            return False
        try:
            self.log.info('referencing tornado z position')

            await self.robot_brain.send(
                f'{self.name}_z_is_referencing = true;'
                f'{self.name}_z.limits({self.speed_limit}, {self.current_limit});'
            )
            await rosys.sleep(0.5)

            # if in end bottom_stop disable end_stop
            if self.end_bottom:
                await self.robot_brain.send(f'{self.name}_end_bottom_enabled = false;')

            # if already in end_top_stop disable and move down
            if self.end_top:
                await self.robot_brain.send(
                    f'{self.name}_end_top_enabled = false;'
                    f'{self.name}_end_bottom_enabled = true;'
                    f'{self.name}_z.move({-0.5*self.speed_limit});'
                )
                await rosys.sleep(0.2)
                while self.end_top:
                    await rosys.sleep(0.2)

            # move up until end top
            await self.robot_brain.send(
                f'{self.name}_end_top_enabled = true;'
                f'{self.name}_z.move({0.5*self.speed_limit});'
            )
            await rosys.sleep(0.2)
            while not self.end_top:
                await rosys.sleep(0.2)

            # move out of end top
            await self.robot_brain.send(
                f'{self.name}_z.move({-0.5*self.speed_limit});'
            )
            await rosys.sleep(0.2)
            while self.end_top:
                await rosys.sleep(0.2)

            # move slowly to end top
            await self.robot_brain.send(
                f'{self.name}_z.move({0.2*self.speed_limit})t;'
            )
            await rosys.sleep(0.2)
            while not self.end_top:
                await rosys.sleep(0.2)

            # move slowly out of end top
            await self.robot_brain.send(
                f'{self.name}_z.move({-0.2*self.speed_limit});'
            )
            await rosys.sleep(0.2)
            while self.end_top:
                await rosys.sleep(0.2)

            # set as zero position
            await self.robot_brain.send(
                f'{self.name}_z.zero();'
                f'{self.name}_z_is_referencing = false;'
            )
            await rosys.sleep(0.2)
            self.log.info('referencing tornado z position done')
            return True
        except Exception as e:
            self.log.error(f'error while referencing tornado z position: {e}')
            return False
        finally:
            await self.robot_brain.send(
                f'{self.name}_z_is_referencing = false;'
                f'{self.name}_end_top_enabled = true;'
                f'{self.name}_end_bottom_enabled = true;'
            )

    async def try_reference_turn(self) -> bool:
        if not await super().try_reference_turn():
            return False
        try:
            self.log.info('referencing tornado turn position')

            await self.robot_brain.send(
                f'{self.name}_turn_is_referencing = true;'
                f'{self.name}_turn.limits({self.speed_limit}, {self.current_limit});'
            )
            await rosys.sleep(0.5)

            # drive to motor ref
            await self.robot_brain.send(
                f'{self.name}_ref_motor_enabled = true;'
                f'{self.name}_turn.move({0.2*self.speed_limit});'
            )
            await rosys.sleep(0.2)
            while not self.ref_motor:
                await rosys.sleep(0.2)

            # drive to gear ref
            await self.robot_brain.send(
                f'{self.name}_ref_gear_enabled = true;'
                f'{self.name}_turn.move({-0.2*self.speed_limit});'
            )
            await rosys.sleep(0.2)
            while not self.ref_gear:
                await rosys.sleep(0.2)

            # set as zero position
            await self.robot_brain.send(
                f'{self.name}_turn.zero();'
                f'{self.name}_turn_is_referencing = false;'
            )
            await rosys.sleep(0.2)
            self.log.info('referencing tornado turn position done')
            return True
        except Exception as e:
            self.log.error(f'error while referencing tornado turn position: {e}')
            return False
        finally:
            await self.robot_brain.send(
                f'{self.name}_turn_is_referencing = false;'
                f'{self.name}_ref_motor_enabled = false;'
                f'{self.name}_ref_gear_enabled = false;'
            )

    def handle_core_output(self, time: float, words: list[str]) -> None:
        self.end_top = int(words.pop(0)) == 0
        self.end_bottom = int(words.pop(0)) == 0
        if self.end_bottom:
            self.is_referenced = False
        self.ref_motor = int(words.pop(0)) == 0
        self.ref_gear = int(words.pop(0)) == 1
        self.ref_t = int(words.pop(0)) == 1
        self.ref_b = int(words.pop(0)) == 1
        self.position_z = float(words.pop(0))
        self.position_turn = (float(words.pop(0)) * 360)


class TornadoSimulation(Tornado, rosys.hardware.ModuleSimulation):

    def __init__(self, *,
                 name: str = 'tornado',
                 min_position: float = 0.1,
                 m_per_tick: float = 0.01,
                 is_z_reversed: bool = False,
                 is_turn_reversed: bool = False,
                 ) -> None:
        self.name = name
        self.m_per_tick = m_per_tick
        self.is_z_reversed = is_z_reversed
        self.is_turn_reversed = is_turn_reversed
        super().__init__(min_position=min_position)

    async def stop_z(self) -> None:
        await super().stop_z()

    async def stop_turn(self) -> None:
        await super().stop_turn()

    async def move_to(self, position: float) -> None:
        try:
            await super().move_to(position)
        except RuntimeError as e:
            raise Exception(e)
        self.position_z = position

    async def move_down_until_reference(self) -> None:
        try:
            await super().move_down_until_reference()
        except RuntimeError as e:
            raise Exception(e)
        self.position_z = self.min_position

    async def turn_by(self, angle: float) -> None:
        try:
            await super().turn_by(angle)
        except RuntimeError as e:
            raise Exception(e)
        self.position_turn = angle

    async def try_reference_z(self) -> bool:
        if not await super().try_reference_z():
            return False
        try:
            self.log.info('referencing tornado z position')
            self.position_z = 0
            self.is_referenced = True
            self.log.info('referencing tornado z position done')
            return True
        except Exception as e:
            self.log.error(f'error while referencing tornado z position: {e}')
            return False

    async def try_reference_turn(self) -> bool:
        if not await super().try_reference_turn():
            return False
        try:
            self.log.info('referencing tornado turn position')
            self.position_turn = 0
            self.is_referenced = True
            self.log.info('referencing tornado turn position done')
            return True
        except Exception as e:
            self.log.error(f'error while referencing tornado turn position: {e}')
            return False
