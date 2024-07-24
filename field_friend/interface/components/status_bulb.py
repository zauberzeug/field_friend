from nicegui import ui


class StatusBulb(ui.element):
    def __init__(self, status: bool) -> None:
        super().__init__(tag='span')
        self.style("height: 15px; width: 15px; margin: auto;   background: radial-gradient(circle at 5px 5px, #5FE5E0, #5898D4); border-radius: 50%; box-shadow: #5FE5E066 0px 0px 10px 5px;") \
            if status else self.style("height: 15px; width: 15px; margin: auto; background: radial-gradient(circle at 5px 5px, #2E5396, #294790); border-radius: 50%;")
