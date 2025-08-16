from nicegui.elements.mixins.value_element import ValueElement


class StatusBulb(ValueElement):
    def __init__(self, value: bool = False) -> None:
        super().__init__(value=value, on_value_change=self.on_change, tag='span')
        self.style('height: 15px; width: 15px; margin: auto; border-radius: 50%')
        self.on_change()

    def on_change(self) -> None:
        self.style('background: radial-gradient(circle at 5px 5px, #5898D4, #4682B4);' if self.value
                   else 'background: radial-gradient(circle at 5px 5px, #D3D3D3, #A9A9A9);')
