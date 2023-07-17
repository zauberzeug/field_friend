import rosys
from nicegui import ui
from nicegui.events import ValueChangeEventArguments

from ..automations import Puncher
from ..hardware import FieldFriend


def axis_control(field_friend: FieldFriend, automator: rosys.automation.Automator, puncher: Puncher) -> None:
    with ui.card(), ui.row():
        with ui.column().classes('items-stretch'):
            ui.markdown('**Software E-Stop**')
            with ui.row().classes('items-center'):
                ui.label('E-Stop').classes('mr-auto')
                ui.button('OFF', on_click=lambda: automator.start(field_friend.robot_brain.send('en3.off()')))
                ui.button('ON', on_click=lambda: automator.start(field_friend.robot_brain.send('en3.on()')))
            with ui.row().classes('items-center'):
                ui.label('RDYP').classes('mr-auto')
                ui.button('OFF', on_click=lambda: automator.start(field_friend.robot_brain.send('rdyp.off()')))
                ui.button('ON', on_click=lambda: automator.start(field_friend.robot_brain.send('rdyp.on()')))

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
        with ui.column():
            ui.markdown('**Punch control**')
            ui.button('Reference all', on_click=lambda: automator.start(puncher.try_home()))
            ui.button('clear view', on_click=lambda: automator.start(puncher.clear_view()))
            depth = ui.number('punch depth', value=0.02, format='%.2f', step=0.01)
            with ui.row():
                ui.button(on_click=lambda: automator.start(
                    puncher.punch(field_friend.y_axis.MAX_POSITION, depth.value)))
                ui.button(on_click=lambda: automator.start(puncher.punch(0, depth.value)))
                ui.button(on_click=lambda: automator.start(
                    puncher.punch(field_friend.y_axis.MIN_POSITION, depth.value)))
