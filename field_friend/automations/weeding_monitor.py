

import rosys

from . import PuncherException, Weeding, WorkflowException


class WeedingMonitor(Weeding):

    async def _perform_workflow(self) -> None:
        self.log.info('Starting Monitoring Workflow...')
        try:
            closest_crop_position = list(self.crops_to_handle.values())[0]
            self.log.info(f'Closest crop position: {closest_crop_position}')
            # fist check if the closest crop is in the working area
            if closest_crop_position.x < self.WORKING_DISTANCE:
                self.log.info(f'target next crop at {closest_crop_position}')
                # do not steer while advancing on a crop
                drive_distance = closest_crop_position.x - self.system.field_friend.WORK_X
                target = self.system.odometer.prediction.transform(rosys.geometry.Point(x=drive_distance, y=0))
                await self.system.driver.drive_to(target)
                self.system.plant_locator.resume()
            else:
                if self.crops_to_handle:
                    await self._follow_line_of_crops()
                else:
                    await self._driving_a_bit_forward()
            await rosys.sleep(0.2)
            self.log.info('workflow completed')
        except PuncherException as e:
            self.log.error(f'Error while Monitoring Workflow: {e}')
        except Exception as e:
            raise WorkflowException(f'Error while Monitoring Workflow: {e}') from e

    def _has_plants_to_handle(self) -> bool:
        super()._has_plants_to_handle()
        return any(self.crops_to_handle)
