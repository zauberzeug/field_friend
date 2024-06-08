import logging
from typing import Any, Literal, Optional, TypedDict

import rosys
from nicegui import events, ui

from ...automations import Field, FieldObstacle, FieldProvider, Row
from ...localization import Gnss
from .geodata_picker import geodata_picker
from .leaflet_map import leaflet_map


class ActiveObject(TypedDict):
    object_type: Literal["Obstacles", "Rows", "Outline"]
    object: Row | FieldObstacle


class field_planner:

    def __init__(self, field_provider: FieldProvider, odometer: rosys.driving.Odometer, gnss: Gnss, leaflet: leaflet_map) -> None:
        self.log = logging.getLogger("field_friend.field_planner")
        self.field_provider = field_provider
        self.odometer = odometer
        self.gnss = gnss
        self.leaflet_map = leaflet
        self.active_field: Field | None = None
        self.active_object: ActiveObject | None = None
        self.tab: Literal["Obstacles", "Outline", "Rows"] = "Outline"

        with ui.row().classes("w-full").style("height: 100%; max-height:100%; width: 100%;"):
            with ui.card().style("width: 48%; max-width: 48%; max-height: 100%; height: 100%;"):
                with ui.row():
                    ui.button("Upload Field", on_click=lambda field_provider=self.field_provider: geodata_picker(field_provider)) \
                        .tooltip("Upload a file with field boundaries. Supported file formats: KML, XML and Shape").classes("ml-auto").style("display: block; margin-top:auto; margin-bottom: auto;")
                    ui.button("Add field", on_click=self.field_provider.create_field).tooltip("Add a new field") \
                        .classes("ml-auto").style("display: block; margin-top:auto; margin-bottom: auto;")
                    ui.button("Clear fields", on_click=self.field_provider.clear_fields).props("outline color=warning") \
                        .tooltip("Delete all fields").classes("ml-auto").style("display: block; margin-top:auto; margin-bottom: auto;")
                with ui.row().style("width: 100%;"):
                    self.show_field_table()
            self.show_field_settings()
            self.show_object_settings()
            self.field_provider.FIELDS_CHANGED.register(self.refresh_ui)

    def set_tab(self, e: events.ValueChangeEventArguments) -> None:
        self.tab = e.value
        self._set_active_object(None)
        self.show_field_settings.refresh()

    def table_selected(self, selection):
        self.active_object = None
        if len(selection.selection) > 0:
            if self.active_field is not None and (selection.selection[0]["id"] == self.active_field.id):
                return
            self._set_active_field(selection.selection[0]["id"])
        else:
            self._set_active_field(None)

    @ui.refreshable
    def show_field_table(self) -> None:
        columns = [
            {"name": "id", "label": "id", "field": "id", "required": True, "align": "center"},
            {"name": "name", "label": "Name", "field": "name", "required": True, "align": "left"},
            {"name": "boundary", "label": "Boundary", "field": "boundary", "sortable": True, "align": "center"},
            {"name": "obstacles", "label": "Obstacles", "field": "obstacles", "sortable": True, "align": "center"},
            {"name": "rows", "label": "Rows", "field": "rows", "sortable": True, "align": "center"},
        ]
        rows = []
        for field in self.field_provider.fields:
            data = {
                "id": field.id,
                "name": field.name,
                "boundary": f"{len(field.points)} points",
                "obstacles": f"{len(field.obstacles)}",
                "rows": f"{len(field.rows)}",
            }
            rows.append(data)
        self.field_table = ui.table(columns=columns, rows=rows, row_key="id", selection="single",
                                    on_select=self.table_selected, pagination=4).style("width: 98%; max-width: 98%;")
        columns[0]["classes"] = "hidden"
        columns[0]["headerClasses"] = "hidden"
        self.field_table.update()
        if self.active_field is not None:
            active_field_data = {
                "id": self.active_field.id,
                "name": self.active_field.name,
                "boundary": f"{len(self.active_field.points)} points",
                "obstacles": f"{len(self.active_field.obstacles)}",
                "rows": f"{len(self.active_field.rows)}",
            }
            self.field_table.selected.append(active_field_data)

    @ui.refreshable
    def show_field_settings(self) -> None:
        with ui.card().style("width: 25%; max-height: 100%; height: 100%;"):
            if self.active_field is None:
                with ui.column().style("display: block; margin: auto;"):
                    ui.icon("fence").props("size=lg color=primary").style("display: block; margin: auto;")
                    ui.label("select a field").style("display: block; margin: auto; color: #6E93D6;")
            else:
                with ui.row().style("width: 100%"):
                    ui.icon("fence").props("size=lg color=primary") \
                        .style("display:block; margin-top:auto; margin-bottom: auto;")
                    ui.input(value=f"{self.active_field.name}") \
                        .on("blur", self.field_provider.invalidate).bind_value(self.active_field, "name").classes("w-32")
                    ui.button(on_click=lambda field=self.active_field: self.field_provider.remove_field(field)) \
                        .props("icon=delete color=warning fab-mini flat").classes("ml-auto").style("display: block; margin-top:auto; margin-bottom: auto;").tooltip("Delete field")
                with ui.tabs().style("width: 100%;") as self.tabs:
                    ui.tab("Outline", "Outline")
                    ui.tab("Obstacles", "Obstacles")
                    ui.tab("Rows", "Rows")
                with ui.tab_panels(self.tabs, value=f"{self.tab}", on_change=self.set_tab).style("width: 100%;") as self.panels:
                    with ui.tab_panel("Outline").style("width: 100%;"):
                        for geo_point in self.active_field.points:
                            with ui.row().style("width: 100%;"):
                                ui.button(on_click=lambda point=geo_point: self.leaflet_map.m.set_center(point.tuple)) \
                                    .props("icon=place color=primary fab-mini flat").tooltip("center map on point").classes("ml-0")
                                ui.number("latitude", value=geo_point.lat, format="%.6f", step=0.1, on_change=lambda event, point=geo_point,
                                          field=self.active_field: self.field_provider.add_field_point(field, point, [event.value, point.long])).classes("w-20")
                                ui.number("longitude", value=geo_point.long, format="%.6f", step=0.1, on_change=lambda event, point=geo_point,
                                          field=self.active_field: self.field_provider.add_field_point(field, point, [point.lat, event.value])).classes("w-20")
                                ui.button(on_click=lambda point=geo_point, field=self.active_field: self.field_provider
                                          .add_field_point(field, point)).props("icon=edit_location_alt color=primary fab-mini flat").tooltip("Relocate point").classes("ml-0")
                                ui.separator()
                        with ui.row().classes("items-center mt-2"):
                            ui.icon("place").props("size=sm color=grey").classes("ml-2")
                            ui.button("", on_click=lambda field=self.active_field: self.field_provider
                                      .add_field_point(field)) \
                                .props("icon=add color=primary fab-mini flat").tooltip("Add point")
                            ui.button("", on_click=lambda field=self.active_field: self.field_provider
                                      .remove_field_point(field)) \
                                .props("icon=remove color=warning fab-mini flat").tooltip("Remove point")

                    with ui.tab_panel("Obstacles"):
                        with ui.row().style("width: 100%;"):
                            radio_el = {}
                            for obstacle in self.active_field.obstacles:
                                radio_el[obstacle.id] = obstacle.name
                            if (self.active_object is not None and self.active_object["object"] is not None and self.tab == "Obstacles"):
                                obstacle_radio = ui.radio(radio_el, on_change=lambda event: self._set_active_object(
                                    event.value, self.tab), value=self.active_object["object"].id)
                            else:
                                obstacle_radio = ui.radio(
                                    radio_el, on_change=lambda event: self._set_active_object(event.value, self.tab))
                        with ui.row().classes("items-center mt-3").style("width: 100%"):
                            ui.button(icon="add", on_click=lambda field=self.active_field: self.field_provider
                                      .create_obstacle(field)).props("color=primary outline").style("width: 100%")
                    with ui.tab_panel("Rows"):
                        with ui.row().style("width: 100%;"):
                            radio_el = {}
                            for row in self.active_field.rows:
                                radio_el[row.id] = row.name
                            if (self.active_object is not None and self.active_object["object"] is not None and self.tab == "Rows"):
                                row_radio = ui.radio(radio_el, on_change=lambda event:
                                                     self._set_active_object(event.value, self.tab), value=self.active_object["object"].id)
                            else:
                                row_radio = ui.radio(
                                    radio_el, on_change=lambda event: self._set_active_object(event.value, self.tab))
                        with ui.row().classes("items-center mt-3").style("width: 100%"):
                            ui.button(icon="add", on_click=lambda field=self.active_field: self.field_provider.create_row(
                                field)).props("color=primary outline").style("width: 100%")
                        with ui.row().classes("items-center mt-3").style("width: 100%"):
                            ui.button("sort rows", on_click=lambda field=self.active_field: self.field_provider.sort_rows(field))

    @ui.refreshable
    def show_object_settings(self) -> None:
        with ui.card().style("width: 24%; max-height: 100%; height: 100%;"):
            if self.active_object is None or self.active_object["object"] is None:
                with ui.column().style("display: block; margin: auto;"):
                    ui.icon("extension").props("size=lg color=primary").style("display: block; margin: auto;")
                    ui.label("select an object").style("display: block; margin: auto; color: #6E93D6;")
            else:
                if self.tab == "Obstacles":
                    with ui.row().style("width: 100%;"):
                        ui.icon("dangerous").props("size=sm color=primary") \
                            .style("display:block; margin-top:auto; margin-bottom: auto;")
                        ui.input("Obstacle name", value=f'{self.active_object["object"].name}',) \
                            .on("blur", self.field_provider.invalidate).bind_value(self.active_object["object"], "name").classes("w-32")
                        ui.button(on_click=lambda field=self.active_field, obstacle=self.active_object["object"]: self.field_provider.remove_obstacle(field, obstacle)) \
                            .props("icon=delete color=warning fab-mini flat").classes("ml-auto").style("display:block; margin-top:auto; margin-bottom: auto;").tooltip("Delete obstacle")
                    with ui.column().style("display: block; overflow: auto; width: 100%"):
                        assert self.active_field is not None
                        assert self.active_field.reference is not None
                        for geo_point in self.active_object["object"].points:
                            with ui.row().style("width: 100%;"):
                                ui.button(on_click=lambda point=geo_point: self.leaflet_map.m.set_center(point.tuple)) \
                                    .props("icon=place color=primary fab-mini flat").tooltip("center map on point").classes("ml-0")
                                ui.number("latitude", value=geo_point.lat, format="%.6f", step=0.1, on_change=lambda event, point=geo_point, field=self.active_field,
                                          obstacle=self.active_object["object"]: self.field_provider.add_obstacle_point(field, obstacle, point, [event.value, point.long])).classes("w-20")
                                ui.number("longitude", value=geo_point.long, format="%.6f", step=0.1, on_change=lambda event, point=geo_point, field=self.active_field,
                                          obstacle=self.active_object["object"]: self.field_provider.add_obstacle_point(field, obstacle, point, [point.lat, event.value])).classes("w-20")
                                ui.button(on_click=lambda point=geo_point, field=self.active_field, obstacle=self.active_object["object"]: self.field_provider
                                          .add_obstacle_point(field, obstacle, point)).props("icon=edit_location_alt color=primary fab-mini flat").tooltip("Relocate point").classes("ml-0")
                        with ui.row().classes("items-center mt-2"):
                            ui.icon("place").props("size=sm color=grey").classes("ml-8")
                            ui.button("", on_click=lambda field=self.active_field, obstacle=self.active_object["object"]: self.field_provider
                                      .add_obstacle_point(field, obstacle),).props("icon=add color=primary fab-mini flat")
                            ui.button("", on_click=lambda obstacle=self.active_object["object"]: self.field_provider.remove_obstacle_point(obstacle)) \
                                .props("icon=remove color=warning fab-mini flat")
                elif self.tab == "Rows":
                    with ui.row().style("width: 100%"):
                        ui.icon("spa").props("size=sm color=primary") \
                            .style("height: 100%; margin-top:auto; margin-bottom: auto;")
                        with ui.column():
                            ui.button(on_click=lambda row=self.active_object["object"]: self.field_provider.move_row(self.active_field, row)) \
                                .props("icon=expand_less color=primary fab-mini flat").style("display:block; margin-top:auto; margin-bottom: auto; margin-left: 0; margin-right: 0;")
                            ui.button(on_click=lambda row=self.active_object["object"]: self.field_provider.move_row(self.active_field, row, next=True)) \
                                .props("icon=expand_more color=primary fab-mini flat").classes("ml-auto").style("display:block; margin-top:auto; margin-bottom: auto; margin-left: 0; margin-right: 0;")
                        ui.input(value=self.active_object["object"].name).on("blur", self.field_provider.invalidate) \
                            .bind_value(self.active_object["object"], "name").classes("w-32")
                        ui.button(on_click=lambda row=self.active_object["object"]: self.field_provider.remove_row(self.active_field, row)) \
                            .props("icon=delete color=warning fab-mini flat").classes("ml-auto").style("display:block; margin-top:auto; margin-bottom: auto;").tooltip("Delete Row")
                    with ui.column().style("display: block; overflow: auto; width: 100%"):
                        for geo_point in self.active_object["object"].points:
                            with ui.row().style("width: 100%;"):
                                ui.button(on_click=lambda point=geo_point: self.leaflet_map.m.set_center(point.tuple)) \
                                    .props("icon=place color=primary fab-mini flat").tooltip("center map on point").classes("ml-0")
                                ui.number("latitude", value=geo_point.lat, format="%.6f", step=0.1, on_change=lambda event, point=geo_point, field=self.active_field, row=self.active_object["object"]: self.field_provider
                                          .add_row_point(field, row, point, [event.value, point[1]]),).classes("w-20")
                                ui.number("longitude", value=geo_point.long, format="%.6f", step=0.1, on_change=lambda event, point=geo_point, field=self.active_field, row=self.active_object["object"]: self.field_provider
                                          .add_row_point(field, row, point, [point[0], event.value]),).classes("w-20")
                                ui.button(on_click=lambda point=geo_point, field=self.active_field, row=self.active_object["object"]: self.field_provider
                                          .add_row_point(field, row, point)).props("icon=edit_location_alt color=primary fab-mini flat").tooltip("Relocate point").classes("ml-0")
                        with ui.row().classes("items-center mt-2").style("display: block; margin: auto;"):
                            ui.icon("place").props("size=sm color=grey").classes("ml-2")
                            ui.button("", on_click=lambda field=self.active_field, row=self.active_object["object"]: self.field_provider.add_row_point(field, row),) \
                                .props("icon=add color=primary fab-mini flat").tooltip("Add point")
                            ui.button("", on_click=lambda row=self.active_object["object"]: self.field_provider.remove_row_point(row),) \
                                .props("icon=remove color=warning fab-mini flat").tooltip("Remove point")

    def refresh_ui(self) -> None:
        # function to call when field updated
        # check if active field and object is still available and rerender otherwise deselect them and set them to none
        if self.active_field is not None:
            if self.active_field not in self.field_provider.fields:
                self.active_field = None
                self.active_object = None
            else:
                if (self.active_object is not None and self.active_object["object"] is not None):
                    if (self.active_object["object"] not in self.active_field.obstacles and self.active_object["object"] not in self.active_field.rows):
                        self.active_object = None
                else:
                    self.active_object = None
        else:
            if self.active_object is not None:
                self.active_object = None
        self.show_field_table.refresh()
        self.show_field_settings.refresh()
        self.show_object_settings.refresh()

    def _set_active_field(self, field_id: str) -> None:
        self.active_field = self.field_provider.get_field(field_id)
        if self.active_field is not None:
            self.show_field_settings.refresh()

    def _set_active_object(self, object_id: Optional[str] = None, object_type: Optional[Literal["Obstacles", "Rows", "Outline"]] = None) -> None:
        if (self.active_field is not None and object_id is not None and object_type is not None):
            if object_type == "Obstacles":
                for obstacle in self.active_field.obstacles:
                    if object_id == obstacle.id:
                        self.active_object = {"object_type": object_type, "object": obstacle}
            elif object_type == "Rows":
                for row in self.active_field.rows:
                    if object_id == row.id:
                        self.active_object = {"object_type": object_type, "object": row}
            else:
                self.active_object = None
        else:
            self.active_object = None
        self.show_object_settings.refresh()
