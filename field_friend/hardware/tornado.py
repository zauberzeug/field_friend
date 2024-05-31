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

        self.last_angle: float = 0.0

        self.is_referenced: bool = False
        self.z_is_referenced: bool = False
        self.turn_is_referenced: bool = False

        self.end_top: bool = False
        self.end_bottom: bool = False

        self.ref_motor: bool = False
        self.ref_gear: bool = False
        self.ref_knife_stop: bool = False
        self.ref_knife_ground: bool = False

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
        if not self.z_is_referenced:
            raise RuntimeError('zaxis is not referenced, reference first')
        if not self.min_position <= position:
            raise RuntimeError(f'zaxis depth is out of range, min: {self.min_position}, given: {position}')

    @abc.abstractmethod
    async def move_down_until_reference(self) -> None:
        if not self.z_is_referenced:
            raise RuntimeError('zaxis is not referenced, reference first')

    @abc.abstractmethod
    async def turn_by(self, turns: float) -> None:
        if not self.turn_is_referenced:
            raise RuntimeError('zaxis is not referenced, reference first')

    @abc.abstractmethod
    async def turn_knifes_to(self, angle: float) -> None:
        if not self.turn_is_referenced:
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
            if not self.is_referenced:
                raise RuntimeError('zaxis is not referenced, reference first')
            await self.move_to(0)
        except RuntimeError as e:
            self.log.error(f'error while returning to reference: {e}')
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
                 end_top_pin_expander: bool = False,
                 end_bottom_pin: int = 5,
                 end_bottom_pin_expander: bool = False,
                 ref_motor_pin: int = 33,
                 ref_motor_pin_expander: bool = False,
                 ref_gear_pin: int = 4,
                 ref_gear_pin_expander: bool = False,
                 ref_knife_stop_pin: int = 35,
                 ref_knife_stop_pin_expander: bool = False,
                 ref_knife_ground_pin: int = 18,
                 ref_knife_ground_pin_expander: bool = False,
                 motors_on_expander: bool = False,
                 end_stops_on_expander: bool = True,
                 is_z_reversed: bool = False,
                 is_turn_reversed: bool = False,
                 speed_limit: int = 1,
                 turn_speed_limit: int = 1,
                 current_limit: int = 20,
                 z_reference_speed: float = 0.0075,
                 turn_reference_speed: float = 0.25,
                 ) -> None:
        self.name = name
        self.expander = expander
        self.speed_limit = speed_limit
        self.turn_speed_limit = turn_speed_limit
        self.current_limit = current_limit
        self.z_reference_speed = z_reference_speed
        self.turn_reference_speed = turn_reference_speed

        lizard_code = remove_indentation(f'''
            {name}_motor_z = {expander.name + "." if motors_on_expander and expander else ""}ODriveMotor({can.name}, {z_can_address})
            {name}_motor_turn = {expander.name + "." if motors_on_expander and expander else ""}ODriveMotor({can.name}, {turn_can_address})
            {name}_motor_z.m_per_tick = {m_per_tick}
            {name}_motor_turn.m_per_tick = {1/12.52}
            {name}_motor_z.limits({self.speed_limit}, {self.current_limit})
            {name}_motor_turn.limits({self.turn_speed_limit}, {self.current_limit})
            {name}_motor_z.reversed = {'true' if is_z_reversed else 'false'}
            {name}_motor_turn.reversed = {'true' if is_turn_reversed else 'false'}
            {name}_end_top = {expander.name + "." if end_stops_on_expander or end_top_pin_expander and expander else ""}Input({end_top_pin})
            {name}_end_top.inverted = true;
            {name}_end_bottom = {expander.name + "." if end_stops_on_expander or end_bottom_pin_expander and expander else ""}Input({end_bottom_pin})
            {name}_end_bottom.inverted = true;
            {name}_ref_motor = {expander.name + "." if end_stops_on_expander or ref_motor_pin_expander and expander else ""}Input({ref_motor_pin})
            {name}_ref_motor.inverted = true;
            {name}_ref_gear = {expander.name + "." if end_stops_on_expander or ref_gear_pin_expander and expander else ""}Input({ref_gear_pin})
            {name}_ref_gear.inverted = false;
            {name}_ref_knife_stop = {expander.name + "." if end_stops_on_expander or ref_knife_stop_pin_expander and expander else ""}Input({ref_knife_stop_pin})
            {name}_ref_knife_stop.inverted = false;
            {name}_ref_knife_ground = {expander.name + "." if end_stops_on_expander or ref_knife_ground_pin_expander and expander else ""}Input({ref_knife_ground_pin})
            {name}_ref_knife_ground.inverted = true;
            {name}_z = {expander.name + "." if motors_on_expander and expander else ""}MotorAxis({name}_motor_z, {name}_end_top, {name}_end_bottom)

            bool {name}_is_referencing = false;
            bool {name}_end_top_enabled = true;
            bool {name}_end_bottom_enabled = true;
            when {name}_end_top_enabled and {name}_is_referencing and {name}_end_top.level == 0 then
                {name}_z.speed(0);
            end
            when !{name}_end_top_enabled and {name}_is_referencing and {name}_end_top.level == 1 then
                {name}_z.speed(0);
            end
            when {name}_end_bottom_enabled and {name}_end_bottom.level == 0 then
                {name}_z.speed(0);
            end
            bool {name}_ref_motor_enabled = false;
            bool {name}_ref_gear_enabled = false;
            when {name}_ref_motor_enabled and {name}_is_referencing and {name}_ref_motor.level == 0 then
                {name}_motor_turn.speed(0);
            end
            when {name}_ref_gear_enabled and {name}_is_referencing and {name}_ref_gear.level == 1 then
                {name}_motor_turn.speed(0);
            end
            bool {name}_knife_ground_enabled = false;
            bool {name}_knife_stop_enabled = false;
            when {name}_knife_ground_enabled and {name}_ref_knife_ground.level == 1 then
                {name}_z.stop();
            end
            when {name}_knife_stop_enabled and {name}_ref_knife_stop.level == 1 then
                en3.off();
                {name}_knife_stop_enabled = false;
            end
        ''')
        core_message_fields = [
            f'{name}_end_top.active',
            f'{name}_end_bottom.active',
            f'{name}_ref_motor.active',
            f'{name}_ref_gear.active',
            f'{name}_ref_knife_stop.active',
            f'{name}_ref_knife_ground.active',
            f'{name}_motor_z.position:3',
            f'{name}_motor_turn.position:3',
        ]
        super().__init__(
            min_position=min_position,
            robot_brain=robot_brain,
            lizard_code=lizard_code,
            core_message_fields=core_message_fields
        )

    async def stop_z(self) -> None:
        await super().stop_z()
        await self.robot_brain.send(f'{self.name}_z.speed(0, 0)')

    async def stop_turn(self) -> None:
        await super().stop_turn()
        await self.robot_brain.send(f'{self.name}_motor_turn.speed(0)')

    async def move_to(self, position: float) -> None:
        try:
            await super().move_to(position)
        except RuntimeError as e:
            raise Exception(e) from e
        await self.robot_brain.send(f'{self.name}_z.position({position}, {self.speed_limit}, 0);')
        while not position - 0.005 < self.position_z < position + 0.005:
            await rosys.sleep(0.1)

    async def move_down_until_reference(self) -> None:
        try:
            await super().move_down_until_reference()
        except RuntimeError as e:
            raise Exception(e) from e
        try:
            self.log.info('moving z axis down')
            # await self.robot_brain.send(
            #     f'{self.name}_knife_stop_enabled = true;'
            #     f'{self.name}_knife_ground_enabled = true;'
            # )
            # await rosys.sleep(0.5)
            await self.robot_brain.send(
                f'{self.name}_z.position({self.min_position}, {self.speed_limit}, 0);'
            )
            await rosys.sleep(0.5)
            while not self.ref_knife_ground and not self.ref_knife_stop:
                if self.min_position - 0.005 <= self.position_z <= self.min_position + 0.005:
                    self.log.info('minimum position reached')
                    break
                self.log.info('moving z axis down')
                await rosys.sleep(0.1)
            if self.ref_knife_stop:
                raise Exception('Error while moving z axis down: Ref knifes stop triggered')
            if self.ref_knife_ground:
                self.log.info('Ref ground triggered: Bottom ground reached')
        except Exception as e:
            self.log.error(f'error while moving z axis down: {e}')
            self.is_referenced = False
            self.z_is_referenced = False
            self.turn_is_referenced = False
            raise Exception(e) from e
        finally:
            # await rosys.sleep(1.5)
            self.log.info('finalizing moving z axis down')
            await self.robot_brain.send(
                f'{self.name}_z.speed(0, 0);'
                # f'{self.name}_knife_stop_enabled = false;'
                # f'{self.name}_knife_ground_enabled = false;'
            )

    async def turn_by(self, turns: float) -> None:
        try:
            await super().turn_by(turns)
        except RuntimeError as e:
            raise Exception(e)
        target = self.position_turn/360 + turns
        await self.robot_brain.send(f'{self.name}_motor_turn.position({target});')
        while not target - 0.01 < self.position_turn/360 < target + 0.01:
            await rosys.sleep(0.1)

    async def turn_knifes_to(self, angle: float) -> None:
        try:
            await super().turn_knifes_to(angle)
        except RuntimeError as e:
            raise Exception(e) from e
        target_angle = angle - self.last_angle
        if target_angle == 0:
            return
        elif target_angle < 0:
            target_angle = 360 - self.last_angle + angle
        target = (self.position_turn - target_angle)/360
        await self.robot_brain.send(f'{self.name}_motor_turn.position({target});')
        while not target - 0.01 < self.position_turn/360 < target + 0.01:
            await rosys.sleep(0.1)
        self.last_angle = angle

    async def try_reference_z(self) -> bool:
        if not await super().try_reference_z():
            return False
        try:
            self.log.info('referencing tornado z position')
            # await self.robot_brain.send(
            #     f'{self.name}_is_referencing = true;'
            #     f'{self.name}_end_top_enabled = true;'
            #     f'{self.name}_end_bottom_enabled = true;'
            # )
            # await rosys.sleep(1)

            # if in end bottom_stop disable end_stop
            if self.end_bottom:
                self.log.info('already in end_bottom, moving out of end bottom stop')
                # await self.robot_brain.send(f'{self.name}_end_bottom_enabled = false;')

            # move up until end top
            if not self.end_top:
                self.log.info('moving up until end top')
                # await self.robot_brain.send(f'{self.name}_end_top_enabled = true;')
                # await rosys.sleep(1)
                await self.robot_brain.send(
                    f'{self.name}_z.speed({self.z_reference_speed}, 0);'
                )
                await rosys.sleep(0.2)
                while not self.end_top:
                    await self.robot_brain.send(f'{self.name}_z.speed({self.z_reference_speed}, 0);')
                    await rosys.sleep(0.2)
                await self.robot_brain.send(f'{self.name}_z.speed(0, 0);')

            # move out of end top
            self.log.info('moving out of end top')
            # await self.robot_brain.send(f'{self.name}_end_top_enabled = false;')
            # await rosys.sleep(1)
            await self.robot_brain.send(f'{self.name}_z.speed({-self.z_reference_speed}, 0);')
            await rosys.sleep(0.2)
            while self.end_top:
                await self.robot_brain.send(f'{self.name}_z.speed({-self.z_reference_speed}, 0);')
                await rosys.sleep(0.2)
            await self.robot_brain.send(f'{self.name}_z.speed(0, 0);')

            # move slowly to end top
            self.log.info('moving slowly to end top')
            # await self.robot_brain.send(f'{self.name}_end_top_enabled = true;')
            # await rosys.sleep(1)
            await self.robot_brain.send(f'{self.name}_z.speed({self.z_reference_speed/5}, 0);')
            await rosys.sleep(0.2)
            while not self.end_top:
                await self.robot_brain.send(f'{self.name}_z.speed({self.z_reference_speed/5}, 0);')
                await rosys.sleep(0.2)
            await self.robot_brain.send(f'{self.name}_z.speed(0, 0);')

            # move slowly out of end top
            self.log.info('moving slowly out of end top')
            # await self.robot_brain.send(f'{self.name}_end_top_enabled = false;')
            # await rosys.sleep(1)
            await self.robot_brain.send(f'{self.name}_z.speed({-self.z_reference_speed/5}, 0);')
            await rosys.sleep(0.2)
            while self.end_top:
                await self.robot_brain.send(f'{self.name}_z.speed({-self.z_reference_speed/5}, 0);')
                await rosys.sleep(0.2)
            await self.robot_brain.send(f'{self.name}_z.speed(0, 0);')

            # set as zero position
            self.log.info('setting as zero position')
            await rosys.sleep(1)
            await self.robot_brain.send(
                f'{self.name}_motor_z.zero();'
                # f'{self.name}_is_referencing = false;'
            )
            self.log.info('referencing tornado z position done')
            self.z_is_referenced = True
            return True
        except Exception as e:
            self.log.error(f'error while referencing tornado z position: {e}')
            return False
        # finally:
        #     await self.robot_brain.send(
        #         f'{self.name}_is_referencing = false;'
        #         f'{self.name}_end_top_enabled = true;'
        #         f'{self.name}_end_bottom_enabled = true;'
        #     )

    async def try_reference_turn(self) -> bool:
        if not await super().try_reference_turn():
            return False
        try:
            self.log.info('referencing tornado turn position')

            await self.robot_brain.send(
                f'{self.name}_is_referencing = true;'
                f'{self.name}_ref_motor_enabled = false;'
                f'{self.name}_ref_gear_enabled = false;'
            )
            await rosys.sleep(0.5)

            # drive to gear ref
            await self.robot_brain.send(f'{self.name}_ref_gear_enabled = true;')
            await rosys.sleep(1)
            await self.robot_brain.send(
                f'{self.name}_motor_turn.speed({self.turn_reference_speed});'
            )
            await rosys.sleep(0.2)
            while not self.ref_gear:
                await self.robot_brain.send(f'{self.name}_motor_turn.speed({self.turn_reference_speed});')
                await rosys.sleep(0.1)
            await self.robot_brain.send(f'{self.name}_motor_turn.speed(0);')

            # drive to motor ref
            await self.robot_brain.send(f'{self.name}_ref_motor_enabled = true;')
            await rosys.sleep(1)
            await self.robot_brain.send(f'{self.name}_motor_turn.move({-self.turn_reference_speed});')
            await rosys.sleep(0.2)
            while not self.ref_motor:
                await self.robot_brain.send(f'{self.name}_motor_turn.speed({-self.turn_reference_speed});')
                await rosys.sleep(0.1)
            await self.robot_brain.send(f'{self.name}_motor_turn.speed(0);')

            # set as zero position
            self.log.info('setting as zero position')
            await rosys.sleep(1)
            await self.robot_brain.send(
                f'{self.name}_motor_turn.zero();'
                f'{self.name}_is_referencing = false;'
            )
            await rosys.sleep(0.2)
            self.log.info('referencing tornado turn position done')
            self.turn_is_referenced = True
            self.last_angle = 0
            return True
        except Exception as e:
            self.log.error(f'error while referencing tornado turn position: {e}')
            return False
        finally:
            await self.robot_brain.send(
                f'{self.name}_is_referencing = false;'
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
        self.ref_knife_stop = int(words.pop(0)) == 1
        self.ref_knife_ground = int(words.pop(0)) == 0
        self.position_z = float(words.pop(0))
        self.position_turn = float(words.pop(0)) * 360


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
            raise Exception(e) from e
        self.position_z = position

    async def move_down_until_reference(self) -> None:
        try:
            await super().move_down_until_reference()
        except RuntimeError as e:
            raise Exception(e) from e
        self.position_z = self.min_position

    async def turn_by(self, angle: float) -> None:
        try:
            await super().turn_by(angle)
        except RuntimeError as e:
            raise Exception(e) from e
        self.position_turn = angle

    async def turn_knifes_to(self, angle: float) -> None:
        try:
            await super().turn_knifes_to(angle)
        except RuntimeError as e:
            raise Exception(e) from e
        self.position_turn = angle

    async def turn_knifes_to(self, angle: float) -> None:
        try:
            await super().turn_knifes_to(angle)
        except RuntimeError as e:
            raise Exception(e)
        self.position_turn = angle

    async def try_reference_z(self) -> bool:
        if not await super().try_reference_z():
            return False
        try:
            self.log.info('referencing tornado z position')
            self.position_z = 0
            self.log.info('referencing tornado z position done')
            self.z_is_referenced = True
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
            self.log.info('referencing tornado turn position done')
            self.turn_is_referenced = True
            return True
        except Exception as e:
            self.log.error(f'error while referencing tornado turn position: {e}')
            return False
