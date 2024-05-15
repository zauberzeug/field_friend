
import logging
from typing import TYPE_CHECKING

from nicegui import app, events, ui

from .automation_controls import automation_controls
from .key_controls import KeyControls
from .leaflet_map import leaflet_map

if TYPE_CHECKING:
    from field_friend.system import System


class operation:

    def __init__(self, system: 'System', leaflet_map: leaflet_map) -> None:
        self.log = logging.getLogger('field_friend.operation')
        self.system = system
        self.field_provider = system.field_provider
        self.field = None
        self.key_controls = KeyControls(self.system)
        self.leaflet_map = leaflet_map
        self.initial_value = None

        with ui.card().tight().classes('w-full').style('margin-bottom: 10px; min-height: 100%;'):
            with ui.row().classes('m-4').style('width: calc(100% - 2rem)'):
                with ui.column().classes('w-full'):
                    with ui.row().classes('items-center'):
                        @ui.refreshable
                        def center_map_button() -> None:
                            if self.field_provider.active_field is not None and len(self.field_provider.active_field.outline_wgs84) > 0:
                                ui.button(on_click=lambda: self.leaflet_map.m.set_center(self.field_provider.active_field.outline_wgs84[0])) \
                                    .props('icon=place color=primary fab-mini flat').tooltip('center map on point').classes('ml-0')
                            else:
                                ui.icon('place').props('size=sm color=grey').classes('ml-2')
                        center_map_button()
                        self.field_provider.FIELD_SELECTED.register(center_map_button.refresh)

                        field_selection_dict = {}
                        if self.field_provider.fields is not None and len(self.field_provider.fields) > 0:
                            for field in self.system.field_provider.fields:
                                field_selection_dict[field.id] = field.name
                            active = self.field_provider.active_field
                            self.initial_value = app.storage.user.get('field') if active is None else active.id
                        self.field_selection = None

                        @ui.refreshable
                        def show_field_selection() -> None:
                            self.field_selection = ui.select(
                                field_selection_dict,
                                with_input=True,
                                on_change=self.set_field,
                                label='Field')\
                                .tooltip('Select the field to work on').classes('w-24')
                            self.field_selection.value = self.initial_value
                        show_field_selection()
                        self.field_provider.FIELDS_CHANGED.register(show_field_selection.refresh)
                    ui.separator()
                    with ui.row():
                        ui.label('Automation').classes('text-xl')
                    with ui.row().classes('w-full'):
                        self.automations_toggle = ui.select([key for key in self.system.automations.keys()],
                                                            on_change=self.handle_automation_changed) \
                            .classes('w-full border pl-2').style('border: 2px solid #6E93D6; border-radius: 5px; background-color: #EEF4FA')
                        self.automations_toggle.value = app.storage.user.get('automation', 'weeding')
                    with ui.column().bind_visibility_from(self.automations_toggle, 'value', value='mowing'):
                        with ui.row():
                            ui.number('Padding', value=0.5, step=0.1, min=0.0, format='%.1f') \
                                .props('dense outlined suffix=m').classes('w-24') \
                                .bind_value(system.mowing, 'padding') \
                                .tooltip('Set the padding for the mowing automation')
                            ui.number('Lane distance', value=0.5, step=0.1, min=0.0, format='%.1f') \
                                .props('dense outlined suffix=m') \
                                .classes('w-24').bind_value(system.mowing,   'lane_distance') \
                                .tooltip('Set the lane distance for the system. automation')
                            ui.number('Number of outer lanes', value=3, step=1, min=3, format='%.0f') \
                                .props('dense outlined').classes('w-28') \
                                .bind_value(system.mowing, 'number_of_outer_lanes') \
                                .tooltip('Set the number of outer lanes for the mowing automation')
                            ui.number('Min. turning radius', format='%.2f', value=0.5, step=0.05, min=0.1, max=2.0) \
                                .props('dense outlined suffix=m') \
                                .classes('w-32') \
                                .bind_value(self.system.mowing, 'minimum_turning_radius') \
                                .tooltip('Set the turning radius for the mowing automation')

                    with ui.column().bind_visibility_from(self.automations_toggle, 'value', value='weeding'):
                        ui.separator()
                        ui.markdown('Field settings').style('color: #6E93D6')
                        with ui.row():
                            self.with_field_planning = ui.checkbox('Use field planning', value=True).bind_value(
                                self.system.weeding, 'use_field_planning').tooltip('Set the weeding automation to use the field planning with GNSS')

                            with ui.row().bind_visibility_from(self.with_field_planning, 'value', value=True):
                                self.show_start_row()
                                self.show_end_row()

                                ui.number('Min. turning radius', format='%.2f',
                                          value=0.5, step=0.05, min=0.05, max=2.0) \
                                    .props('dense outlined suffix=m').classes('w-30') \
                                    .bind_value(self.system.weeding, 'minimum_turning_radius') \
                                    .tooltip('Set the turning radius for the weeding automation')
                                ui.number('turn_offset', format='%.2f', value=0.4, step=0.05, min=0.05, max=2.0) \
                                    .props('dense outlined suffix=m').classes('w-30') \
                                    .bind_value(self.system.weeding, 'turn_offset') \
                                    .tooltip('Set the turning offset for the weeding automation')
                                ui.checkbox('Drive backwards to start', value=True).bind_value(self.system.weeding, 'drive_backwards_to_start') \
                                    .tooltip('Set the weeding automation to drive backwards to the start row at the end of the row')
                                ui.checkbox('Drive to start row', value=True).bind_value(self.system.weeding, 'drive_to_start') \
                                    .tooltip('Set the weeding automation to drive to the start of the row before starting the weeding')

                        ui.separator()
                        ui.markdown('Detector settings').style('color: #6E93D6')
                        with ui.row():
                            ui.number('Min. weed confidence', format='%.2f', value=0.8, step=0.05, min=0.0, max=1.0) \
                                .props('dense outlined') \
                                .classes('w-24') \
                                .bind_value(self.system.plant_locator, 'minimum_weed_confidence') \
                                .tooltip('Set the minimum weed confidence for the weeding automation')
                            ui.number('Min. crop confidence', format='%.2f', value=0.4, step=0.05, min=0.0, max=1.0) \
                                .props('dense outlined') \
                                .classes('w-24') \
                                .bind_value(self.system.plant_locator, 'minimum_crop_confidence') \
                                .tooltip('Set the minimum crop confidence for the weeding automation')
                        ui.separator()
                        ui.markdown('Tool settings').style('color: #6E93D6')
                        with ui.row():
                            if self.system.field_friend.tool == 'tornado':
                                ui.number('Tornado angle', format='%.0f', value=180, step=1, min=1, max=180) \
                                    .props('dense outlined suffix=°') \
                                    .classes('w-24') \
                                    .bind_value(self.system.weeding, 'tornado_angle') \
                                    .tooltip('Set the angle for the tornado drill')
                            elif self.system.field_friend.tool in ['weed_screw', 'dual_mechanism']:
                                ui.number('Drill depth', value=0.02, format='%.2f', step=0.01,
                                          min=self.system.field_friend.z_axis.max_position, max=self.system.field_friend.z_axis.min_position*-1) \
                                    .props('dense outlined suffix=°') \
                                    .classes('w-24') \
                                    .bind_value(self.system.weeding, 'weed_screw_depth') \
                                    .tooltip('Set the drill depth for the weeding automation')
                                ui.number('Crop safety distance', value=0.01, step=0.001, min=0.001, max=0.05, format='%.3f') \
                                    .props('dense outlined suffix=m') \
                                    .classes('w-24') \
                                    .bind_value(self.system.weeding, 'crop_safety_distance') \
                                    .tooltip('Set the crop safety distance for the weeding automation')
                        ui.separator()
                        ui.markdown('Workflow settings').style('color: #6E93D6')
                        with ui.row():
                            ui.checkbox('Only monitoring') \
                                .bind_value(self.system.weeding, 'only_monitoring') \
                                .tooltip('Set the weeding automation to only monitor the field')
                            if self.system.field_friend.tool == 'tornado':
                                ui.checkbox('With punch check', value=True) \
                                    .bind_value(self.system.puncher, 'with_punch_check') \
                                    .tooltip('Set the weeding automation to check for punch')
                                ui.checkbox('Drill 2x with open torando', value=False,) \
                                    .bind_value(self.system.weeding, 'drill_with_open_tornado') \
                                    .tooltip('Set the weeding automation to drill a second time with open tornado')
                                ui.checkbox('Drill between crops', value=False) \
                                    .bind_value(self.system.weeding, 'drill_between_crops') \
                                    .tooltip('Set the weeding automation to drill between crops')
                            elif self.system.field_friend.tool == 'dual_mechanism':
                                ui.checkbox('Drilling', value=False).bind_value(self.system.weeding, 'with_drilling') \
                                    .tooltip('Set the weeding automation to with drill')
                                ui.checkbox('Chopping', value=False) \
                                    .bind_value(self.system.weeding, 'with_chopping') \
                                    .tooltip('Set the weeding automation to with chop')
                                ui.checkbox('Chop if no crops', value=False) \
                                    .bind_value(self.system.weeding, 'chop_if_no_crops') \
                                    .tooltip('Set the weeding automation to chop also if no crops seen')
                        ui.separator()
                        ui.markdown('**Driver settings**').style('color: #6E93D6')
                        with ui.row():
                            ui.number('linear_speed_on_row', value=0.5, step=0.005, min=0.015, format='%.2f') \
                                .props('dense outlined suffix=m/s') \
                                .classes('w-24') \
                                .bind_value(self.system.weeding, 'linear_speed_on_row') \
                                .tooltip('Set the linear speed on row for the weeding automation')
                            ui.number('linear_speed_between_rows', value=0.5, step=0.01, min=0.03, format='%.2f') \
                                .props('dense outlined suffix=m/s') \
                                .classes('w-24') \
                                .bind_value(self.system.weeding, 'linear_speed_between_rows') \
                                .tooltip('Set the linear speed between rows for the weeding automation')
                            ui.number('angular_speed_on_row', value=0.5, step=0.01, min=0.03, format='%.2f') \
                                .props('dense outlined suffix=°/s') \
                                .classes('w-24') \
                                .bind_value(self.system.weeding, 'angular_speed_on_row') \
                                .tooltip('Set the angular speed on row for the weeding automation')
                            ui.number('angular_speed_between_rows', value=0.5, step=0.01, min=0.03, format='%.2f') \
                                .props('dense outlined suffix=°/s') \
                                .classes('w-24') \
                                .bind_value(self.system.weeding, 'angular_speed_between_rows') \
                                .tooltip('Set the angular speed between rows for the weeding automation')

                    with ui.column().bind_visibility_from(self.automations_toggle, 'value', value='monitoring'):
                        with ui.column():
                            self.with_field_planning_monitor = ui.checkbox('Use field planning', value=True) \
                                .bind_value(self.system.monitoring, 'use_field_planning') \
                                .tooltip('Set the monitoring automation to use the field planning with GNSS')

                        with ui.row().bind_visibility_from(self.with_field_planning_monitor, 'value', value=True):
                            self.show_start_row()
                            self.show_end_row()
                            ui.number('Min. turning radius', format='%.2f',
                                      value=0.5, step=0.05, min=0.05, max=2.0) \
                                .props('dense outlined suffix=m').classes('w-30') \
                                .bind_value(self.system.monitoring, 'minimum_turning_radius') \
                                .tooltip('Set the turning radius for the monitoring automation')

                    with ui.column().bind_visibility_from(self.automations_toggle, 'value', value='collecting (demo)'):
                        with ui.row():
                            ui.number('Drill angle', format='%.0f', value=100, step=1, min=1, max=180) \
                                .props('dense outlined suffix=°').classes('w-24') \
                                .bind_value(self.system.coin_collecting, 'angle') \
                                .tooltip('Set the drill depth for the weeding automation')
                            ui.checkbox('with drilling', value=True) \
                                .bind_value(self.system.coin_collecting, 'with_drilling')
            ui.space()
            with ui.row().style("margin: 1rem; width: calc(100% - 2rem);"):
                with ui.column():
                    ui.button('emergency stop', on_click=lambda: system.field_friend.estop.set_soft_estop(True)).props('color=red') \
                        .classes('py-3 px-6 text-lg').bind_visibility_from(system.field_friend.estop, 'is_soft_estop_active', value=False)
                    ui.button('emergency reset', on_click=lambda: system.field_friend.estop.set_soft_estop(False)) \
                        .props('color=red-700 outline').classes('py-3 px-6 text-lg') \
                        .bind_visibility_from(system.field_friend.estop, 'is_soft_estop_active', value=True)
                ui.space()
                with ui.row():
                    automation_controls(self.system, can_start=self.can_start)
        with ui.dialog() as self.dialog, ui.card():
            self.dialog_label = ui.label('Do you want to continue the canceled automation').classes('text-lg')
            with ui.row():
                ui.button('Yes', on_click=lambda: self.dialog.submit('Yes'))
                ui.button('No', on_click=lambda: self.dialog.submit('No'))
                ui.button('Cancel', on_click=lambda: self.dialog.submit('Cancel'))

        self.system.puncher.POSSIBLE_PUNCH.register(self.can_punch)
        with ui.dialog() as self.punch_dialog, ui.card():
            self.punch_dialog_label = ui.label('Do you want to punch at the current position?').classes('text-lg')
            with ui.row():
                ui.button('Yes', on_click=lambda: self.punch_dialog.submit('Yes'))
                ui.button('No', on_click=lambda: self.punch_dialog.submit('No'))
                ui.button('Cancel', on_click=lambda: self.punch_dialog.submit('Cancel'))

    @ui.refreshable
    def show_start_row(self) -> None:
        if self.field_provider.active_field is not None:
            ui.select({row.id: row.name for row in self.field_provider.active_field.rows}, label='Start row') \
                .bind_value(self.system.weeding, 'start_row_id').classes('w-24').tooltip('Select the row to start on')
        else:
            ui.select([None], label='Start row')\
                .bind_value(self.system.weeding, 'start_row').classes('w-24').tooltip('Select the row to start on')

    @ui.refreshable
    def show_end_row(self) -> None:
        if self.field_provider.active_field is not None:
            ui.select({row.id: row.name for row in self.field_provider.active_field.rows}, label='End row') \
                .bind_value(self.system.weeding, 'end_row_id').classes('w-24').tooltip('Select the row to end on')
        else:
            ui.select([None], label='End row') \
                .bind_value(self.system.weeding, 'end_row').classes('w-24').tooltip('Select the row to end on')

    def set_field(self) -> None:
        for field in self.system.field_provider.fields:
            if field.id == self.field_selection.value:
                self.field_provider.select_field(field)
                app.storage.user['field'] = field.id
                if len(field.outline_wgs84) > 0:
                    self.system.gnss.set_reference(field.outline_wgs84[0][0], field.outline_wgs84[0][1])
                # TODO das hier noch auf das active field umbauen, damit auch diese werte im weeding auf das active field registriert sind
                self.system.weeding.field = field
                self.system.mowing.field = field
                self.show_start_row.refresh()
                self.show_end_row.refresh()

    async def can_punch(self) -> None:
        self.punch_dialog_label.text = 'Do you want to punch at the current position?'
        result = await self.punch_dialog
        if result == 'Yes':
            self.system.puncher.punch_allowed = 'allowed'
        elif result == 'No':
            self.system.puncher.punch_allowed = 'not_allowed'
        elif result == 'Cancel':
            self.system.puncher.punch_allowed = 'not_allowed'

    async def can_start(self) -> bool:
        self.log.info('Checking if automation can be started')
        if self.automations_toggle.value == 'mowing':
            return await self.can_mowing_start()
        elif self.automations_toggle.value == 'weeding':
            return await self.can_weeding_start()
        return True

    async def can_mowing_start(self) -> bool:
        self.log.info('Checking mowing automation')
        if self.system.mowing.current_path_segment is None:
            self.system.mowing.continue_mowing = False
            return True
        self.dialog_label.text = 'Do you want to continue the canceled mowing automation?'
        result = await self.dialog
        if result == 'Yes':
            self.system.mowing.continue_mowing = True
        elif result == 'No':
            self.system.mowing.continue_mowing = False
        elif result == 'Cancel':
            return False
        return True

    async def can_weeding_start(self) -> bool:
        self.log.info('Checking weeding automation')
        if not self.system.weeding.current_row or not self.system.weeding.current_segment:
            self.system.weeding.continue_canceled_weeding = False
            return True
        self.dialog_label.text = f'Do you want to continue the canceled weeding automation on row {self.system.weeding.current_row.name}?'
        result = await self.dialog
        if result == 'Yes':
            self.system.weeding.continue_canceled_weeding = True
        elif result == 'No':
            self.system.weeding.continue_canceled_weeding = False
        elif result == 'Cancel':
            return False
        return True

    def handle_automation_changed(self, e: events.ValueChangeEventArguments) -> None:
        app.storage.user.update({'automation': e.value})
        self.system.automator.default_automation = self.system.automations[e.value]
