
import logging
from typing import TYPE_CHECKING

from nicegui import ui

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
                                ui.button(on_click=lambda: self.leaflet_map.m.set_center(self.field_provider.active_field.outline_wgs84[0])).props(
                                    'icon=place color=primary fab-mini flat').tooltip('center map on point').classes('ml-0')
                            else:
                                ui.icon('place').props('size=sm color=grey').classes('ml-2')
                        center_map_button()
                        self.field_provider.FIELD_SELECTED.register(center_map_button.refresh)

                        field_selection_dict = {}
                        if self.field_provider.fields is not None and len(self.field_provider.fields) > 0:
                            for field in self.system.field_provider.fields:
                                field_selection_dict[field.id] = field.name
                            self.initial_value = None if self.field_provider.active_field is None else self.field_provider.active_field.id
                        self.field_selection = None

                        @ui.refreshable
                        def show_field_selection() -> None:
                            self.field_selection = ui.select(
                                field_selection_dict,
                                with_input=True, on_change=self.set_field, label='Field', value=self.initial_value).tooltip(
                                'Select the field to work on').classes('w-24')
                        show_field_selection()
                        self.field_provider.FIELDS_CHANGED.register(show_field_selection.refresh)
                    ui.separator()
                    with ui.row():
                        ui.label("Automation").classes('text-xl')
                    with ui.row().classes('w-full'):
                        self.automations_toggle = ui.select(
                            [key for key in self.system.automations.keys()],
                            value='weeding').bind_value(
                            self.system.automator, 'default_automation', forward=lambda key: self.system.automations[key],
                            backward=lambda automation: next(
                                key for key, value in self.system.automations.items() if value == automation)).classes('w-full border pl-2').style('border: 2px solid #6E93D6; border-radius: 5px; background-color: #EEF4FA')

                    with ui.column().bind_visibility_from(self.automations_toggle, 'value', value='mowing'):
                        with ui.row():
                            ui.number('Padding', value=0.5, step=0.1, min=0.0, format='%.1f').props('dense outlined suffix=m').classes(
                                'w-24').bind_value(system.mowing, 'padding').tooltip('Set the padding for the mowing automation')
                            ui.number('Lane distance', value=0.5, step=0.1, min=0.0, format='%.1f').props('dense outlined suffix=m').classes(
                                'w-24').bind_value(system.mowing, 'lane_distance').tooltip('Set the lane distance for the system. automation')
                            ui.number('Number of outer lanes', value=3, step=1, min=3, format='%.0f').props('dense outlined').classes(
                                'w-28').bind_value(system.mowing, 'number_of_outer_lanes').tooltip('Set the number of outer lanes for the mowing automation')
                            ui.number('Min. turning radius', format='%.2f', value=0.5, step=0.1, min=0.1, max=1.0).props(
                                'dense outlined suffix=m').classes('w-32').bind_value(
                                self.system.mowing, 'turning_radius').tooltip(
                                'Set the turning radius for the mowing automation')

                    with ui.column().bind_visibility_from(self.automations_toggle, 'value', value='weeding'):
                        with ui.column():
                            self.with_field_planning = ui.checkbox('Use field planning', value=True).bind_value(
                                self.system.weeding, 'use_field_planning').tooltip('Set the weeding automation to use the field planning with GNSS')

                            with ui.row().bind_visibility_from(self.with_field_planning, 'value', value=True):
                                self.show_start_row()
                                self.show_end_row()
                                ui.number('Min. turning radius', format='%.2f', value=0.5, step=0.1, min=0.1, max=1.0).props(
                                    'dense outlined suffix=m').classes('w-30').bind_value(
                                    self.system.weeding, 'turning_radius').tooltip(
                                    'Set the turning radius for the weeding automation')
                            with ui.row():
                                ui.number('Tornado angle', format='%.0f', value=180, step=1, min=1, max=180).props(
                                    'dense outlined suffix=°').classes('w-24').bind_value(
                                    self.system.weeding, 'tornado_angle').tooltip(
                                    'Set the angle for the tornado drill')
                                ui.checkbox('Only monitoring').bind_value(
                                    self.system.weeding, 'only_monitoring').tooltip(
                                    'Set the weeding automation to only monitor the field')

                    with ui.column().bind_visibility_from(self.automations_toggle, 'value', value='monitoring'):
                        with ui.column():
                            self.with_field_planning_monitor = ui.checkbox('Use field planning', value=True).bind_value(
                                self.system.monitoring, 'use_field_planning').tooltip('Set the monitoring automation to use the field planning with GNSS')

                            with ui.row().bind_visibility_from(self.with_field_planning_monitor, 'value', value=True):
                                self.show_start_row()
                                self.show_end_row()
                                ui.number('Min. turning radius', format='%.2f', value=0.5, step=0.1, min=0.1, max=1.0).props(
                                    'dense outlined suffix=m').classes('w-30').bind_value(
                                    self.system.monitoring, 'turning_radius').tooltip(
                                    'Set the turning radius for the monitoring automation')

                    with ui.column().bind_visibility_from(self.automations_toggle, 'value', value='collecting (demo)'):
                        with ui.row():
                            ui.number(
                                'Drill angle', format='%.0f', value=100, step=1, min=1, max=180).props(
                                'dense outlined suffix=°').classes('w-24').bind_value(
                                self.system.coin_collecting, 'angle').tooltip(
                                'Set the drill depth for the weeding automation')
                            ui.checkbox('with drilling', value=True).bind_value(
                                self.system.coin_collecting, 'with_drilling')

                    with ui.column().bind_visibility_from(self.automations_toggle, 'value', value='followme'):
                        with ui.grid(columns=4):
                            ui.number('Ignore y-Distance', step=1, min=0, max=1080, format='%d').props('dense outlined suffix=px').classes(
                                'w-28').bind_value(self.system.followme, 'max_y_distance').tooltip('Points with a y-distance greater than this value will be ignored')
                            ui.number('First Matching Distance', step=1, min=0, format='%d').props('dense outlined suffix=px').classes(
                                'w-28').bind_value(self.system.followme, 'max_first_matching_distance').tooltip('Maximum allowed distance from the bottom center of the image')
                            ui.number('Matching Distance', step=1, min=0, format='%d').props('dense outlined suffix=px').classes(
                                'w-28').bind_value(self.system.followme, 'max_matching_distance').tooltip('Maximum allowed distance from the last known target position')
                            ui.number('Projection Factor', step=0.1, min=0.0, max=1.0, format='%.1f').props('dense outlined').classes(
                                'w-28').bind_value(self.system.followme, 'projection_factor').tooltip('Scales the contribution of the y-distance to the total distance')
                            ui.number('Stop Distance', step=1, min=0.0, format='%d').props('dense outlined suffix=px').classes(
                                'w-28').bind_value(self.system.followme, 'stop_distance').tooltip('How close the robot should move to the target')
                            ui.number('Min Yaw', step=0.05, min=0.0, max=1.0, format='%.2f').props('dense outlined').classes(
                                'w-28').bind_value(self.system.followme, 'yaw_min').tooltip('TODO: yaw_min')
                            ui.number('Max Yaw', step=0.05, min=0.0, max=1.0, format='%.2f').props('dense outlined').classes(
                                'w-28').bind_value(self.system.followme, 'yaw_max').tooltip('TODO: yaw_max')
                            ui.number('Speed', format='%.2f', step=0.1, min=0.1, max=3.0).props(
                                'dense outlined suffix=m/s').classes('w-28').bind_value(self.system.followme, 'linear_speed').tooltip('Speed when in DRIVE state')
                            ui.number('Turn Speed', format='%.2f', step=0.1, min=0.1, max=3.0).props(
                                'dense outlined suffix=rad/s').classes('w-28').bind_value(self.system.followme, 'angular_speed').tooltip('Rotation speed when in TURN state')
                            ui.number('Min Confidence', format='%.1f', step=0.1, min=0.0, max=1.0).props(
                                'dense outlined').classes('w-28').bind_value(self.system.followme, 'confidence').tooltip('TODO: confidence')
                            ui.checkbox('Drive').bind_value(self.system.followme, 'drive').tooltip('TODO: drive')
                        with ui.grid(columns=4):
                            ui.label().classes('w-1/4').bind_text_from(self.system.followme, 'state',
                                                                       lambda state: f'State: {state}'.replace('FollowState.', ''))
                            ui.label().classes('w-1/4').bind_text_from(self.system.followme,
                                                                       'target', lambda target: f'Target: ({target.x:.1f}, {target.y:.1f})' if target is not None else 'Target: None')
                            ui.label().classes('w-1/4').bind_text_from(self.system.followme,
                                                                       'distance', lambda distance: f'Distance: {distance:.1f}px')
                            ui.label().classes('w-1/4').bind_text_from(self.system.followme,
                                                                       'distance_y', lambda distance_y: f'Distance_Y: {distance_y:.1f}px')
                            ui.label().classes('w-1/4').bind_text_from(self.system.followme,
                                                                       'pixel_percentage_yaw', lambda yaw: f'Yaw: {yaw:.2f}')
                            ui.label().classes('w-1/4').bind_text_from(self.system.followme,
                                                                       'n_feet', lambda n_feet: f'n_Feet: {n_feet}')
                            ui.label().classes('w-1/4').bind_text_from(self.system.followme,
                                                                       'seconds_since_update', lambda seconds_since_update: f'Last Update: {seconds_since_update:.1f}s')

            ui.space()
            with ui.row().style("margin: 1rem; width: calc(100% - 2rem);"):
                with ui.column():
                    ui.button('emergency stop', on_click=lambda: system.field_friend.estop.set_soft_estop(True)).props('color=red').classes(
                        'py-3 px-6 text-lg').bind_visibility_from(system.field_friend.estop, 'is_soft_estop_active', value=False)
                    ui.button('emergency reset', on_click=lambda: system.field_friend.estop.set_soft_estop(False)).props(
                        'color=red-700 outline').classes('py-3 px-6 text-lg').bind_visibility_from(system.field_friend.estop,
                                                                                                   'is_soft_estop_active', value=True)
                ui.space()
                with ui.row():
                    automation_controls(self.system, can_start=self.ensure_start)
        with ui.dialog() as self.dialog, ui.card():
            ui.label(f'Do you want to continue the canceled {"mowing"  if self.automations_toggle.value == "mowing" else f"weeding on {self.system.weeding.current_row.name}"}?').classes(
                'text-lg')
            with ui.row():
                ui.button('Yes', on_click=lambda: self.dialog.submit('Yes'))
                ui.button('No', on_click=lambda: self.dialog.submit('No'))
                ui.button('Cancel', on_click=lambda: self.dialog.submit('Cancel'))

    @ui.refreshable
    def show_start_row(self) -> None:
        if self.field_provider.active_field is not None:
            ui.select({row.id: row.name for row in self.field_provider.active_field.rows}, label='Start row').bind_value(self.system.weeding, 'start_row_id').classes(
                'w-24').tooltip('Select the row to start on')
        else:
            ui.select([None], label='Start row').bind_value(self.system.weeding, 'start_row').classes(
                'w-24').tooltip('Select the row to start on')

    @ui.refreshable
    def show_end_row(self) -> None:
        if self.field_provider.active_field is not None:
            ui.select({row.id: row.name for row in self.field_provider.active_field.rows}, label='End row').bind_value(self.system.weeding, 'end_row_id').classes(
                'w-24').tooltip('Select the row to end on')
        else:
            ui.select([None], label='End row').bind_value(self.system.weeding, 'end_row').classes(
                'w-24').tooltip('Select the row to end on')

    def set_field(self) -> None:
        for field in self.system.field_provider.fields:
            if field.id == self.field_selection.value:
                self.field_provider.select_field(field)
                if len(field.outline_wgs84) > 0:
                    self.system.gnss.set_reference(field.outline_wgs84[0][0], field.outline_wgs84[0][1])
                # TODO das hier noch auf das active field umbauen, damit auch diese werte im weeding auf das active field registriert sind
                self.system.weeding.field = field
                self.system.mowing.field = field
                self.show_start_row.refresh()
                self.show_end_row.refresh()

    async def ensure_start(self) -> bool:
        self.log.info('Ensuring start of automation')
        if self.automations_toggle.value == 'mowing':
            return await self.ensure_mowing_start()
        elif self.automations_toggle.value == 'weeding':
            return await self.ensure_weeding_start()
        return True

    async def ensure_mowing_start(self) -> bool:
        self.log.info('Ensuring start of mowing automation')
        if self.system.mowing.current_path_segment is None:
            return True
        result = await self.dialog
        if result == 'Yes':
            self.system.mowing.continue_mowing = True
        elif result == 'No':
            self.system.mowing.continue_mowing = False
        elif result == 'Cancel':
            return False
        return True

    async def ensure_weeding_start(self) -> bool:
        self.log.info('Ensuring start of weeding automation')
        if self.system.weeding.current_segment is None:
            return True
        result = await self.dialog
        if result == 'Yes':
            self.system.weeding.continue_canceled_weeding = True
        elif result == 'No':
            self.system.weeding.continue_canceled_weeding = False
        elif result == 'Cancel':
            return False
        return True
