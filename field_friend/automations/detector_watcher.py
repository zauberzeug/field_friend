import logging

import rosys
from nicegui import ui


class DetectorWatcher:
    CHECK_CHARGING_INTERVAL = 5

    def __init__(self, bms: rosys.hardware.Bms, *,
                 plant_detector: rosys.vision.Detector | None = None,
                 circle_sight_detector: rosys.vision.Detector | None = None) -> None:
        self.log = logging.getLogger('field_friend.detector_watcher')
        self.bms = bms
        self.plant_detector = plant_detector
        self.circle_sight_detector = circle_sight_detector

        self.active = True
        self.last_charging_state = False
        rosys.on_startup(self._pause_on_startup)
        rosys.on_repeat(self._check_charging, self.CHECK_CHARGING_INTERVAL)

    async def _pause_on_startup(self) -> None:
        while isinstance(self.plant_detector, rosys.vision.DetectorHardware) and not self.plant_detector.is_connected:
            await rosys.sleep(0.1)
        while isinstance(self.circle_sight_detector, rosys.vision.DetectorHardware) and not self.circle_sight_detector.is_connected:
            await rosys.sleep(0.1)
        await self._pause_detectors()

    async def _check_charging(self) -> None:
        if not self.active:
            return
        if self.bms.state is None or self.bms.state.is_charging is None:
            return
        if self.bms.state.is_charging == self.last_charging_state:
            return
        if self.bms.state.is_charging and not self.last_charging_state:
            await self._update_detectors()
        elif not self.bms.state.is_charging and self.last_charging_state:
            await self._pause_detectors()
        self.last_charging_state = self.bms.state.is_charging

    async def _update_detectors(self) -> None:
        self.log.debug('setting detectors to follow_loop')
        if self.plant_detector is not None:
            await self.plant_detector.set_model_version('follow_loop')
        if self.circle_sight_detector is not None:
            await self.circle_sight_detector.set_model_version('follow_loop')

    async def _pause_detectors(self) -> None:
        self.log.debug('pausing detectors')
        if self.plant_detector is not None:
            await self.plant_detector.set_model_version('pause')
        if self.circle_sight_detector is not None:
            await self.circle_sight_detector.set_model_version('pause')

    def developer_ui(self) -> None:
        async def handle_version_control_change(value: str) -> None:
            if value == 'Follow Loop':
                self.active = False
                await self._update_detectors()
            elif value == 'Pause':
                self.active = False
                await self._pause_detectors()
            elif value == 'Auto':
                self.active = True

        ui.label('Detector Watcher').classes('text-center text-bold')
        with ui.column().classes('w-32'):
            options = ['Follow Loop', 'Pause', 'Auto']
            ui.select(label='Version Control', options=options, value='Auto' if self.active else 'Pause',
                      on_change=lambda e: handle_version_control_change(e.value)).classes('w-full') \
                .tooltip('Auto: Follow Loop if charging, Pause if not charging')
