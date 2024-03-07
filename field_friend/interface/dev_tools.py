from datetime import timedelta

import psutil
import rosys
from nicegui import ui

from field_friend.system import System

from ..hardware import (ChainAxis, FieldFriend, FieldFriendHardware, FlashlightPWMHardware, Tornado, YAxis,
                        YAxisTornado, ZAxis, ZAxisV2)
from ..navigation import Gnss
from .development import development
from .hardware_control import hardware_control
from .status_dev_page import status_dev_page


def dev_tools(system: System) -> None:
    with ui.card().style('background-color: #2E5396; width: 100%;'):
        with ui.column().style("width: 100%;"):
            ui.label("Development Tools").style('font-size: 1.5rem; color: white;')
            with ui.row().style("width: 100%"):
                development(system.field_friend)
                hardware_control(system.field_friend, system.automator, system.puncher)
                status_dev_page(system.field_friend, system.gnss, system.odometer)
