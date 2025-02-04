import rosys
from nicegui import ui
from nicegui.events import ValueChangeEventArguments

from ...automations import Puncher
from ...hardware import (
    Axis,
    ChainAxis,
    FieldFriend,
    FieldFriendHardware,
    FlashlightPWM,
    FlashlightPWMHardwareV2,
    FlashlightPWMV2,
    FlashlightV2,
    Tornado,
    YAxisCanOpenHardware,
    ZAxisCanOpenHardware,
)


def create_hardware_control_ui(field_friend: FieldFriend, automator: rosys.automation.Automator, puncher: Puncher) -> None:
    with ui.card().style('background-color: #3E63A6; color: white;'), ui.row():
        with ui.column().classes('items-stretch'):
            if isinstance(field_friend, FieldFriendHardware):
                ui.label('Robot Brain Control').classes('font-bold')
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
                ui.label('Flashlight').classes('font-bold')

                async def toggle_flashlight(e: ValueChangeEventArguments) -> None:
                    assert field_friend.flashlight is not None
                    if not isinstance(field_friend.flashlight, FlashlightV2):
                        if e.value == field_friend.flashlight.is_active:
                            return
                        if e.value:
                            await field_friend.flashlight.activate(10)
                            rosys.notify('Flashlight turned on')
                        else:
                            await field_friend.flashlight.deactivate()
                            rosys.notify('Flashlight turned off')

                    elif isinstance(field_friend.flashlight, FlashlightV2 | FlashlightPWM | FlashlightPWMV2):
                        if e.value:
                            await field_friend.flashlight.turn_on()
                            rosys.notify('Flashlight turned on')
                        else:
                            await field_friend.flashlight.turn_off()
                            rosys.notify('Flashlight turned off')

                if not isinstance(field_friend.flashlight, FlashlightV2):
                    ui.switch('On for 10s', on_change=toggle_flashlight) \
                        .bind_value_from(field_friend.flashlight, 'is_active')
                else:
                    ui.switch('Turn On/Off', on_change=toggle_flashlight)

                if isinstance(field_friend.flashlight, FlashlightPWMHardwareV2):
                    ui.slider(min=0, max=1, step=0.01, on_change=field_friend.flashlight.set_duty_cycle) \
                        .bind_value(field_friend.flashlight, 'duty_cycle')

        if field_friend.y_axis is not None:
            if isinstance(field_friend.y_axis, Axis):
                with ui.column():
                    ui.label('Y-Axis').classes('font-bold')
                    ui.button('Reference', on_click=lambda: automator.start(field_friend.y_axis.try_reference()))
                    ui.button('Move to min', on_click=lambda: automator.start(
                        field_friend.y_axis.move_to(field_friend.y_axis.min_position)))
                    ui.button('Move to middle', on_click=lambda: automator.start(
                        field_friend.y_axis.move_to(0)))
                    ui.button('Move to max', on_click=lambda: automator.start(
                        field_friend.y_axis.move_to(field_friend.y_axis.max_position)))
                    if isinstance(field_friend.y_axis, YAxisCanOpenHardware):
                        ui.button('Reset Fault', on_click=lambda: automator.start(field_friend.y_axis.reset_fault()))
            elif isinstance(field_friend.y_axis, ChainAxis):
                with ui.column():
                    ui.label('Chain-Axis').classes('font-bold')
                    ui.button('Reference', on_click=lambda: automator.start(field_friend.y_axis.try_reference()))
                    ui.button('Reset', on_click=lambda: automator.start(field_friend.y_axis.reset()))
                    ui.button('Return to r ref', on_click=lambda: automator.start(
                        field_friend.y_axis.return_to_r_ref()))
                    ui.button('Return to l ref', on_click=lambda: automator.start(
                        field_friend.y_axis.return_to_l_ref()))
                    ui.button('chop to left', on_click=lambda: automator.start(field_friend.y_axis.move_dw_to_l_ref()))
                    ui.button('chop to right', on_click=lambda: automator.start(field_friend.y_axis.move_dw_to_r_ref()))

        if field_friend.z_axis is not None:
            if isinstance(field_friend.z_axis, Axis):
                with ui.column():
                    ui.label('Z-Axis').classes('font-bold')
                    ui.button('Reference', on_click=lambda: automator.start(field_friend.z_axis.try_reference()))
                    ui.button('Return to reference', on_click=lambda: automator.start(
                        field_friend.z_axis.return_to_reference()))
                    ui.button('Move to min', on_click=lambda: automator.start(
                        field_friend.z_axis.move_to(field_friend.z_axis.min_position)))
                    if isinstance(field_friend.z_axis, ZAxisCanOpenHardware):
                        ui.button('Reset Fault', on_click=lambda: automator.start(field_friend.z_axis.reset_fault()))
            elif isinstance(field_friend.z_axis, Tornado):
                with ui.column():
                    ui.label('Z-Axis').classes('font-bold')
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
                ui.label('Punch control').classes('font-bold')
                ui.button('Reference all', on_click=lambda: automator.start(puncher.try_home()))
                ui.button('clear view', on_click=lambda: automator.start(puncher.clear_view()))
                if isinstance(field_friend.y_axis, ChainAxis):
                    ui.button('chop', on_click=lambda: automator.start(puncher.chop()))
                if isinstance(field_friend.z_axis, Tornado):
                    angle = ui.number('angle', value=180, format='%.0f', step=1,
                                      min=0, max=180).classes('w-24').style('background-color: white; padding: 0.5rem; border-radius: 5px;')
                else:
                    depth = ui.number('punch depth', value=0.02, format='%.2f', step=0.01, min=field_friend.z_axis.max_position, max=field_friend.z_axis.min_position*-1).classes('w-24').style(
                        'background-color: white; padding: 0.5rem; border-radius: 5px;')
                with ui.row():
                    if isinstance(field_friend.y_axis, ChainAxis):
                        ui.button(on_click=lambda: automator.start(
                            puncher.punch(field_friend.y_axis.min_position, depth=depth.value)))
                        ui.button(on_click=lambda: automator.start(puncher.punch(0, depth=depth.value)))
                        ui.button(on_click=lambda: automator.start(
                            puncher.punch(field_friend.y_axis.max_position, depth=depth.value)))
                    elif isinstance(field_friend.y_axis, Axis) and isinstance(field_friend.z_axis, Tornado):
                        ui.button(on_click=lambda: automator.start(
                            puncher.punch(field_friend.y_axis.min_position, angle=angle.value)))
                        ui.button(on_click=lambda: automator.start(puncher.punch(0, angle=angle.value)))
                        ui.button(on_click=lambda: automator.start(
                            puncher.punch(field_friend.y_axis.max_position, angle=angle.value)))
                    elif isinstance(field_friend.y_axis, Axis) and isinstance(field_friend.z_axis, Axis):
                        ui.button(on_click=lambda: automator.start(
                            puncher.punch(field_friend.y_axis.max_position, depth=depth.value)))
                        ui.button(on_click=lambda: automator.start(puncher.punch(0, depth=depth.value)))
                        ui.button(on_click=lambda: automator.start(
                            puncher.punch(field_friend.y_axis.min_position - field_friend.WORK_Y, depth=depth.value)))
