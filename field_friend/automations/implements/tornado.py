
from typing import TYPE_CHECKING, Any

import rosys

from ..puncher import PuncherException
from .weeding_implement import ImplementException, WeedingImplement

if TYPE_CHECKING:
    from system import System


class Tornado(WeedingImplement):

    def __init__(self, system: 'System') -> None:
        super().__init__('Tornado', system)
        self.drill_with_open_tornado: bool = False
        self.drill_between_crops: bool = False

    async def _perform_workflow(self) -> None:
        self.log.info('Performing Tornado Workflow..')
        try:
            closest_crop_id, closest_crop_position = list(self.crops_to_handle.items())[0]
            self.log.info(f'Closest crop position: {closest_crop_position}')
            # fist check if the closest crop is in the working area
            if closest_crop_position.x < self.system.field_friend.WORK_X + self.WORKING_DISTANCE:
                self.log.info(f'target next crop at {closest_crop_position}')
                # do not steer while advancing on a crop

                if not self.only_monitoring and self.system.field_friend.can_reach(closest_crop_position) \
                        and not self._crops_in_drill_range(closest_crop_id, closest_crop_position, self.tornado_angle):
                    self.log.info('drilling crop')
                    open_drill = False
                    if self.drill_with_open_tornado and not self._crops_in_drill_range(closest_crop_id, closest_crop_position, 0):
                        open_drill = True
                    await self.system.puncher.drive_and_punch(
                        plant_id=closest_crop_id, x=closest_crop_position.x, y=closest_crop_position.y, angle=self.tornado_angle, with_open_tornado=open_drill, with_punch_check=self.with_punch_check)
                    # if self.drill_with_open_tornado and not self._crops_in_drill_range(closest_crop_id, closest_crop_position, 0):
                    #     self.log.info('drilling crop with open tornado')
                    #     await self.system.puncher.punch(plant_id=closest_crop_id, y=closest_crop_position.y, angle=0)
                else:
                    self.log.info('Cant reach crop')
                    await self._follow_line_of_crops()

                if len(self.crops_to_handle) > 1 and self.drill_between_crops:
                    self.log.info('checking for second closest crop')
                    second_closest_crop_position = list(self.crops_to_handle.values())[1]
                    distance_to_next_crop = closest_crop_position.distance(second_closest_crop_position)
                    if distance_to_next_crop > 0.13:
                        # get the target of half the distance between the two crops
                        target = closest_crop_position.x + distance_to_next_crop / 2
                        self.log.info(f'driving to position between two crops: {target}')
                        if not self.only_monitoring:
                            # punch in the middle position with closed knifes
                            await self.system.puncher.drive_and_punch(plant_id=closest_crop_id, x=target, y=0, angle=180)
                        else:
                            drive_distance = target - self.system.field_friend.WORK_X
                            target = self.system.odometer.prediction.transform(
                                rosys.geometry.Point(x=drive_distance, y=0))
                            await self.system.driver.drive_to(target)

            else:
                await self._follow_line_of_crops()
            await rosys.sleep(0.2)
            self.log.info('workflow completed')
        except PuncherException as e:
            self.log.error(f'Error while Tornado Workflow: {e}')
        except Exception as e:
            raise ImplementException(f'Error while tornado Workflow: {e}') from e

    def backup(self) -> dict:
        return super().backup() | {
            'drill_with_open_tornado': self.drill_with_open_tornado,
            'drill_between_crops': self.drill_between_crops,
        }

    def restore(self, data: dict[str, Any]) -> None:
        super().restore(data)
        self.drill_with_open_tornado = data.get('drill_with_open_tornado', self.drill_with_open_tornado)
        self.drill_between_crops = data.get('drill_between_crops', self.drill_between_crops)

    def _has_plants_to_handle(self) -> bool:
        super()._has_plants_to_handle()
        return any(self.crops_to_handle)
