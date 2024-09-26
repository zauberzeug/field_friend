from typing import TYPE_CHECKING

import rosys
from nicegui import app, ui

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

    async def content(self) -> None:
        await ui.context.client.connected()
        with ui.row().classes('w-full'):
            with ui.column():
                # TODO: remember last selected tab
                with ui.tabs() as tabs:
                    ui.tab('software', label='Software', icon='terminal')
                    ui.tab('hardware', label='Hardware', icon='memory')
                with ui.tab_panels(tabs, value='software').bind_value(app.storage.tab, 'dev_page_tab'):
                    with ui.tab_panel('software'):
                        self.software_ui()
                    with ui.tab_panel('hardware'):
                        self.hardware_ui()

    def software_ui(self) -> None:
        with ui.row():
            with ui.card():
                localization_ui_refreshable = ui.refreshable(localization_ui.developer_ui)
                localization_ui_refreshable(self.system)
                ui.timer(rosys.config.ui_update_interval, lambda: localization_ui_refreshable.refresh(self.system))

            with ui.card():
                kpi_ui_refreshable = ui.refreshable(self.system.kpi_provider.developer_ui)
                kpi_ui_refreshable()
                ui.timer(rosys.config.ui_update_interval, kpi_ui_refreshable.refresh)

            if not isinstance(self.system.field_friend, rosys.hardware.RobotHardware):
                with ui.card():
                    rosys.simulation_ui()

    def hardware_ui(self) -> None:
        with ui.row():
            if isinstance(self.system.field_friend, rosys.hardware.RobotHardware):
                with ui.card():
                    with ui.row():
                        with ui.column():
                            self.system.field_friend.robot_brain.developer_ui()
                        with ui.column():
                            self.system.field_friend.robot_brain.communication.debug_ui()
            hardware_control(self.system.field_friend, self.system.automator, self.system.puncher)
            status_dev_page(self.system.field_friend, self.system)

            with ui.card():
                battery_ui_refreshable = ui.refreshable(battery.developer_ui)
                battery_ui_refreshable(self.system)
                ui.timer(rosys.config.ui_update_interval, lambda: battery_ui_refreshable.refresh(self.system))

        with ui.row():
            io_overview(self.system)
