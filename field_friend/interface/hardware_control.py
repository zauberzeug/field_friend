import rosys
from nicegui import ui
from nicegui.events import ValueChangeEventArguments

from ..automations import Puncher
from ..hardware import (ChainAxis, FieldFriend, FieldFriendHardware, Flashlight, FlashlightPWM, FlashlightV2, HPortal,
                        Tornado, YAxis, YAxisTornado, ZAxis, ZAxisV2)


def hardware_control(field_friend: FieldFriend, automator: rosys.automation.Automator, puncher: Puncher) -> None:
    with ui.card(), ui.row():
        with ui.column().classes('items-stretch'):
            if isinstance(field_friend, FieldFriendHardware):
                ui.markdown('**Robot Brain Control**')
                with ui.row().classes('items-center'):
                    ui.label('EN3').classes('mr-auto')
                    ui.button('OFF', on_click=lambda: automator.start(field_friend.robot_brain.send('en3.off()')))
                    ui.button('ON', on_click=lambda: automator.start(field_friend.robot_brain.send('en3.on()')))
                with ui.row().classes('items-center'):
                    ui.label('RDYP').classes('mr-auto')
                    ui.button('OFF', on_click=lambda: automator.start(field_friend.robot_brain.send('rdyp.off()')))
                    ui.button('ON', on_click=lambda: automator.start(field_friend.robot_brain.send('rdyp.on()')))
                if hasattr(field_friend, 'battery_control') and field_friend.battery_control is not None:
                    with ui.row().classes('items-center'):
                        ui.label('Battery_relais').classes('mr-auto')
                        ui.button('OFF', on_click=lambda: automator.start(
                            field_friend.robot_brain.send(f'{field_friend.battery_control.name}_reset.off()')))
                        ui.button('ON', on_click=lambda: automator.start(
                            field_friend.robot_brain.send(f'{field_friend.battery_control.name}_reset.on()')))
        if field_friend.flashlight is not None:
            with ui.column():
                ui.markdown('**Flashlight**')

                if isinstance(field_friend.flashlight, Flashlight):
                    async def toggle_flashlight(e: ValueChangeEventArguments) -> None:
                        if e.value == field_friend.flashlight.is_active:
                            return
                        if e.value:
                            await field_friend.flashlight.activate(10)
                            rosys.notify('Flashlight turned on')
                        else:
                            await field_friend.flashlight.deactivate()
                            rosys.notify('Flashlight turned off')

                    ui.switch(
                        'On for 10s', on_change=toggle_flashlight).bind_value_from(
                        field_friend.flashlight, 'is_active')
                elif isinstance(field_friend.flashlight, FlashlightV2) or isinstance(field_friend.flashlight, FlashlightPWM):
                    async def toggle_flashlight(e: ValueChangeEventArguments) -> None:
                        if e.value:
                            await field_friend.flashlight.turn_on()
                            rosys.notify('Flashlight turned on')
                        else:
                            await field_friend.flashlight.turn_off()
                            rosys.notify('Flashlight turned off')

                ui.switch('Turn On/Off', on_change=toggle_flashlight)

        if field_friend.y_axis is not None:
            if isinstance(field_friend.y_axis, YAxis):
                with ui.column():
                    ui.markdown('**Y-Axis**')

                    async def toggle_end_stops(e: ValueChangeEventArguments) -> None:
                        if e.value == field_friend.y_axis.end_stops_enabled:
                            return
                        if e.value:
                            await field_friend.y_axis.enable_end_stops(True)
                            rosys.notify('Y-Axis end stops enabled')
                        else:
                            await field_friend.y_axis.enable_end_stops(False)
                            rosys.notify('Y-Axis end stops disabled')
                    ui.button('Reference', on_click=lambda: automator.start(field_friend.y_axis.try_reference()))
                    ui.switch(
                        'Enable end stops', on_change=toggle_end_stops).bind_value_from(
                        field_friend.y_axis, 'end_stops_enabled').disable()
            elif isinstance(field_friend.y_axis, ChainAxis):
                with ui.column():
                    ui.markdown('**Chain-Axis**')
                    ui.button('Reference', on_click=lambda: automator.start(field_friend.y_axis.try_reference()))
                    ui.button('Reset', on_click=lambda: automator.start(field_friend.y_axis.reset()))
                    ui.button('Return to r ref', on_click=lambda: automator.start(
                        field_friend.y_axis.return_to_r_ref()))
                    ui.button('Return to l ref', on_click=lambda: automator.start(
                        field_friend.y_axis.return_to_l_ref()))
                    ui.button('chop to left', on_click=lambda: automator.start(field_friend.y_axis.move_dw_to_l_ref()))
                    ui.button('chop to right', on_click=lambda: automator.start(field_friend.y_axis.move_dw_to_r_ref()))
            elif isinstance(field_friend.y_axis, YAxisTornado):
                with ui.column():
                    ui.markdown('**Y-Axis**')
                    ui.button('Reference', on_click=lambda: automator.start(field_friend.y_axis.try_reference()))
            elif isinstance(field_friend.y_axis, HPortal):
                with ui.column():
                    ui.markdown('**H-Portal**')
                    ui.button('Reference', on_click=lambda: automator.start(field_friend.y_axis.try_reference()))
                    with ui.row().style('width:15em'):
                        # width/height of area accessible to h-portal
                        hp_range_y = field_friend.y_axis.MAX_Y - field_friend.y_axis.MIN_Y - 0.01
                        hp_range_x = field_friend.y_axis.MAX_X - field_friend.y_axis.MIN_X
                        punch_buttons = 3
                        punch_step_y = hp_range_y / (punch_buttons-1)
                        punch_step_x = hp_range_x / (punch_buttons-1)
                        for i in range(0, punch_buttons, 1):
                            for j in range(0, punch_buttons, 1):
                                x = (punch_buttons-1-i) * punch_step_x + field_friend.y_axis.MIN_X
                                y = (punch_buttons-1-j) * punch_step_y + field_friend.y_axis.MIN_Y + 0.005
                                ui.button(icon='sym_o_my_location', on_click=lambda _, x=x, y=y: automator.start(
                                    puncher.aim_with_h_portal(x, y)))

        if field_friend.z_axis is not None:
            if isinstance(field_friend.z_axis, ZAxis) or isinstance(field_friend.z_axis, ZAxisV2):
                with ui.column():
                    ui.markdown('**Z-Axis**')

                    async def toggle_z_ref(e: ValueChangeEventArguments) -> None:
                        if e.value == field_friend.z_axis.is_ref_enabled:
                            return
                        if e.value:
                            await field_friend.z_axis.enable_ref(True)
                            rosys.notify('Z-Axis ref enabled')
                        else:
                            await field_friend.z_axis.enable_ref(False)
                            rosys.notify('Z-Axis ref disabled')

                    async def toggle_end_b(e: ValueChangeEventArguments) -> None:
                        if e.value == field_friend.z_axis.is_end_b_enabled:
                            return
                        if e.value:
                            await field_friend.z_axis.enable_end_stop(True)
                            rosys.notify('Z-Axis end_b enabled')
                        else:
                            await field_friend.z_axis.enable_end_stop(False)
                            rosys.notify('Z-Axis end_b disabled')

                    ui.button('Reference', on_click=lambda: automator.start(field_friend.z_axis.try_reference()))
                    ui.button('Return to reference', on_click=lambda: automator.start(
                        field_friend.z_axis.return_to_reference()))
                    ui.switch('Enable ref stop', on_change=toggle_z_ref).bind_value_from(
                        field_friend.z_axis, 'is_ref_enabled').disable()
                    ui.switch(
                        'Enable end stop', on_change=toggle_end_b).bind_value_from(
                        field_friend.z_axis, 'is_end_b_enabled').disable()
            elif isinstance(field_friend.z_axis, Tornado):
                with ui.column():
                    ui.markdown('**Z-Axis**')
                    ui.button('Reference', on_click=lambda: automator.start(field_friend.z_axis.try_reference()))
                    ui.button('Reference Z', on_click=lambda: automator.start(field_friend.z_axis.try_reference_z()))
                    ui.button('Reference Turn', on_click=lambda: automator.start(
                        field_friend.z_axis.try_reference_turn()))
                    ui.button('Return to reference', on_click=lambda: automator.start(
                        field_friend.z_axis.return_to_reference()))
                    ui.button('down until reference', on_click=lambda: automator.start(
                        field_friend.z_axis.move_down_until_reference()))

        if field_friend.z_axis is not None and field_friend.y_axis is not None:
            with ui.column():
                ui.markdown('**Punch control**')
                ui.button('Reference all', on_click=lambda: automator.start(puncher.try_home()))
                ui.button('clear view', on_click=lambda: automator.start(puncher.clear_view()))
                if isinstance(field_friend.y_axis, ChainAxis):
                    ui.button('chop', on_click=lambda: automator.start(puncher.chop()))
                if isinstance(field_friend.z_axis, Tornado):
                    angle = ui.number('angle', value=180, format='%.0f', step=1, min=0, max=180)
                else:
                    depth = ui.number('punch depth', value=0.02, format='%.2f', step=0.01, min=0.01, max=0.18)
                with ui.row():
                    if isinstance(field_friend.y_axis, ChainAxis):
                        ui.button(on_click=lambda: automator.start(
                            puncher.punch(field_friend.y_axis.MAX_POSITION-field_friend.y_axis.WORK_OFFSET, depth.value)))
                        ui.button(on_click=lambda: automator.start(puncher.punch(0, depth.value)))
                        ui.button(on_click=lambda: automator.start(
                            puncher.punch(field_friend.y_axis.MIN_POSITION+field_friend.y_axis.WORK_OFFSET, depth.value)))
                    elif isinstance(field_friend.y_axis, YAxis):
                        ui.button(on_click=lambda: automator.start(
                            puncher.punch(field_friend.y_axis.MAX_POSITION, depth.value)))
                        ui.button(on_click=lambda: automator.start(puncher.punch(0, depth.value)))
                        ui.button(on_click=lambda: automator.start(
                            puncher.punch(field_friend.y_axis.MIN_POSITION, depth.value)))
                    elif isinstance(field_friend.y_axis, YAxisTornado):
                        ui.button(on_click=lambda: automator.start(
                            puncher.punch(field_friend.y_axis.min_position, angle=angle.value)))
                        ui.button(on_click=lambda: automator.start(puncher.punch(0, angle=angle.value)))
                        ui.button(on_click=lambda: automator.start(
                            puncher.punch(field_friend.y_axis.max_position, angle=angle.value)))
