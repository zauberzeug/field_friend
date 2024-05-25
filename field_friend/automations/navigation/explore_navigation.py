from .navigation import Navigation


class ExploreNavigation(Navigation):

    async def _start(self):
        self.log.info('Weeding without a plan...')
        already_explored_count = 0
        while True:
            self.system.plant_locator.pause()
            self.system.plant_provider.clear()
            if not self.system.is_real:
                self.system.detector.simulated_objects = []
                await self._create_simulated_plants()
            if self.system.field_friend.tool != 'none':
                await self.system.puncher.clear_view()
            await self.system.field_friend.flashlight.turn_on()
            await rosys.sleep(2)
            self.system.plant_locator.resume()
            await rosys.sleep(0.5)
            while self._has_plants_to_handle():
                await self._handle_plants()
                already_explored_count = 0
                await rosys.sleep(0.2)
            if not self._has_plants_to_handle() and already_explored_count < 5:
                self.log.info('No crops found, advancing a bit to ensure there are really no more crops')
                target = self.system.odometer.prediction.transform(Point(x=0.10, y=0))
                await self.system.driver.drive_to(target)
                already_explored_count += 1
            else:
                self.log.info('No more crops found')
                break
