from __future__ import annotations

from typing import TYPE_CHECKING

from .header_bar import HeaderBar as header_bar
from .status_drawer import create_status_drawer

if TYPE_CHECKING:
    from ...system import System


def create_header(system: System) -> None:
    drawer = create_status_drawer(system)
    header_bar(system, drawer)
