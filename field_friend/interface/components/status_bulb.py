from typing import Any, Callable, Optional

from nicegui import ui
from nicegui.elements.mixins.value_element import ValueElement


class StatusBulb(ValueElement):
    def __init__(self, value: bool = False) -> None:
        super().__init__(value=value, on_value_change=self.on_change, tag='span')
        self.style("height: 15px; width: 15px; margin: auto; border-radius: 50%")
        self.on_change()

    def on_change(self) -> None:
        self.style(add="background: radial-gradient(circle at 5px 5px, #5FE5E0, #5898D4); box-shadow: #5FE5E066 0px 0px 10px 5px;" if self.value else "background: radial-gradient(circle at 5px 5px, #2E5396, #294790);")
