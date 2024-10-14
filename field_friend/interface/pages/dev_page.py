from typing import TYPE_CHECKING, Callable

import rosys
from nicegui import app, ui
from rosys.hardware import ESPPins

from ...localization import localization_ui
from ..components import battery, hardware_control, io_overview, status_dev_page

if TYPE_CHECKING:
    from field_friend.system import System


class dev_page():

    def __init__(self, page_wrapper, system: 'System') -> None:
        self.system = system

        @ui.page('/dev')
        async def page() -> None:
            page_wrapper()
            await self.content()

    def setup_refreshable(self, function: Callable, **kwargs) -> None:
        refreshable = ui.refreshable(function)
        refreshable(**kwargs)
        ui.timer(rosys.config.ui_update_interval, lambda: refreshable.refresh(**kwargs))

    async def content(self) -> None:
        await ui.context.client.connected()
        with ui.row().classes('w-full'):
            with ui.column():
                with ui.tabs() as tabs:
                    ui.tab('software', label='Software', icon='terminal')
                    ui.tab('hardware', label='Hardware', icon='memory')
                    ui.tab('robotbrain', label='Robot Brain', icon='psychology')
                    ui.tab('battery', label='Battery', icon='battery_charging_full')
                    ui.tab('plantmap', label='Plant Map', icon='yard')
                with ui.tab_panels(tabs, value='software').bind_value(app.storage.tab, 'dev_page_tab'):
                    with ui.tab_panel('software'):
                        self.software_ui()
                    with ui.tab_panel('hardware'):
                        self.hardware_ui()
                    with ui.tab_panel('robotbrain'):
                        self.robot_brain_ui()
                    with ui.tab_panel('battery'):
                        self.battery_ui()
                    with ui.tab_panel('plantmap'):
                        self.plantmap_ui()

    def software_ui(self) -> None:
        with ui.row():
            with ui.card():
                self.setup_refreshable(localization_ui.developer_ui, system=self.system)
            with ui.card():
                self.setup_refreshable(self.system.kpi_provider.developer_ui)
            if not isinstance(self.system.field_friend, rosys.hardware.RobotHardware):
                with ui.card():
                    rosys.simulation_ui()

    def hardware_ui(self) -> None:
        with ui.row():
            hardware_control(self.system.field_friend, self.system.automator, self.system.puncher)
            status_dev_page(self.system.field_friend, self.system)
        with ui.row():
            io_overview(self.system)

    def robot_brain_ui(self) -> None:
        with ui.row():
            if isinstance(self.system.field_friend, rosys.hardware.RobotHardware):
                with ui.card():
                    with ui.row():
                        with ui.column():
                            self.system.field_friend.robot_brain.developer_ui()
                        with ui.column():
                            self.system.field_friend.robot_brain.communication.debug_ui()
        with ui.row():
            if isinstance(self.system.field_friend, rosys.hardware.RobotHardware):
                with ui.card().style('min-width: 200px; background-color: #3E63A6; color: white;'):
                    esp_pins_core = ESPPins(name='core', robot_brain=self.system.field_friend.robot_brain)
                    esp_pins_core.developer_ui()
                    # self.setup_refreshable(esp_pins_core.developer_ui)
                with ui.card().style('min-width: 200px; background-color: #3E63A6; color: white;'):
                    esp_pins_p0 = ESPPins(name='p0', robot_brain=self.system.field_friend.robot_brain)
                    esp_pins_p0.developer_ui()
                    # self.setup_refreshable(esp_pins_p0.developer_ui)

    def battery_ui(self) -> None:
        with ui.row():
            with ui.card():
                self.setup_refreshable(battery.developer_ui, system=self.system)
            with ui.card():
                ui.label('TODO: Battery History')

    def plantmap_ui(self) -> None:
        ui.label('TODO: Plant Map')
