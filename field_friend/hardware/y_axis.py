import abc
from typing import Optional

import rosys


class YAxis(rosys.hardware.Module, abc.ABC):
    """The y axis module is a simple example for a representation of real or simulated robot hardware."""

    def __init__(self, max_speed, reference_speed, min_position, max_position, axis_offset, steps_per_m, reversed_direction, **kwargs) -> None:
        super().__init__(**kwargs)

        self.max_speed: int = max_speed
        self.reference_speed: int = reference_speed
        self.min_position: float = min_position
        self.max_position: float = max_position
        self.axis_offset: float = axis_offset
        self.steps_per_m: float = steps_per_m
        self.reversed_direction: bool = reversed_direction

        self.steps: int = 0
        self.alarm: bool = False
        self.idle: bool = False

        self.is_referenced: bool = False

        self.end_l: bool = False
        self.end_r: bool = False

        rosys.on_shutdown(self.stop)

    @abc.abstractmethod
    async def stop(self) -> None:
        pass

    @abc.abstractmethod
    async def move_to(self, position: float, speed: int | None = None) -> None:
        if not speed:
            speed = self.max_speed
        if not self.is_referenced:
            raise RuntimeError('yaxis is not referenced, reference first')
        if speed > self.max_speed:
            raise RuntimeError(f'yaxis speed is too high, max speed is {self.max_speed}')
        if not self.min_position <= position <= self.max_position:
            raise RuntimeError(
                f'target position {position} ist out of yaxis range {self.min_position} to {self.max_position}')
        self.log.info(f'moving yaxis to {position} with speed {speed}')

    @abc.abstractmethod
    async def try_reference(self) -> bool:
        return True

    def compute_steps(self, position: float) -> int:
        """Compute the number of steps to move the y axis to the given position.   

        The position is given in meters.
        """
        return int((position + self.axis_offset) * self.steps_per_m) * (-1 if self.reversed_direction else 1)

    def compute_position(self, steps: int) -> float:
        return steps / self.steps_per_m - self.axis_offset * (-1 if self.reversed_direction else 1)

    @property
    def position(self) -> float:
        return self.compute_position(self.steps)


class YAxisSimulation(YAxis, rosys.hardware.ModuleSimulation):
    '''The y axis simulation module is a simple example for a representation of simulated robot hardware.
    '''

    def __init__(self,
                 max_speed: int = 80_000,
                 reference_speed: int = 40,
                 min_position: float = -0.12,
                 max_position: float = 0.12,
                 axis_offset: float = 0.123,
                 steps_per_m: float = 666.67 * 1000,
                 reversed_direction: bool = False,
                 ) -> None:
        self.speed: int = 0
        self.target_steps: Optional[int] = None
        super().__init__(
            max_speed=max_speed,
            reference_speed=reference_speed,
            min_position=min_position,
            max_position=max_position,
            axis_offset=axis_offset,
            steps_per_m=steps_per_m,
            reversed_direction=reversed_direction,
        )

    async def stop(self) -> None:
        await super().stop()
        self.speed = 0
        self.target_steps = None

    async def move_to(self, position: float, speed: int | None = None) -> None:
        if speed is None:
            speed = self.max_speed
        try:
            await super().move_to(position, speed)
        except RuntimeError as e:
            rosys.notify(e, type='negative')
            self.log.error(f'could not move yaxis to {position} because of {e}')
            return
        self.target_steps = self.compute_steps(position)
        self.speed = speed if self.target_steps > self.steps else speed * -1
        while self.target_steps is not None:
            await rosys.sleep(0.2)

    async def try_reference(self) -> bool:
        if not await super().try_reference():
            return False
        self.steps = 0
        self.is_referenced = True
        return True

    async def reset_fault(self) -> None:
        self.alarm = False

    async def step(self, dt: float) -> None:
        await super().step(dt)
        self.steps += int(dt * self.speed)
        self.idle = self.speed == 0
        if self.target_steps is not None:
            if (self.speed > 0) == (self.steps > self.target_steps):
                self.steps = self.target_steps
                self.target_steps = None
                self.speed = 0
