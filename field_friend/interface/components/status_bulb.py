from nicegui import ui


class StatusBulb(ui.element):
    def __init__(self, status: bool) -> None:
        super().__init__(tag='span')
        self.style("height: 15px; width: 15px; margin: auto;   background: radial-gradient(circle at 5px 5px, #EF4444, #9A3333); border-radius: 50%; box-shadow: rgba(255, 0, 0, 0.4) 0px 0px 10px 5px;") \
            if status else self.style("height: 15px; width: 15px; margin: auto; background: radial-gradient(circle at 5px 5px, #2E5396, #294790); border-radius: 50%;")

# def status_bulb(status: bool) -> None:
#     if status:
#         ui.element('span').style(
#             "height: 15px; width: 15px; margin: auto;   background: radial-gradient(circle at 5px 5px, #EF4444, #9A3333); border-radius: 50%; box-shadow: rgba(255, 0, 0, 0.4) 0px 0px 10px 5px;")
#     else:
#         ui.element('span').style(
#             "height: 15px; width: 15px; margin: auto; background: radial-gradient(circle at 5px 5px, #2E5396, #294790); border-radius: 50%;")
