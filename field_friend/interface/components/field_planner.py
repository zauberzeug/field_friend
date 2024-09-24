import logging
from typing import TYPE_CHECKING, Literal, Optional, TypedDict

from nicegui import app, events, ui

from ...automations import Field, FieldObstacle, Row
from .field_creator import FieldCreator
from .geodata_importer import geodata_importer
from .leaflet_map import leaflet_map

if TYPE_CHECKING:
    from field_friend.system import System


TabType = Literal["Plants", "Obstacles", "Outline", "Rows"]


class ActiveObject(TypedDict):
    object_type: Literal["Obstacles", "Rows", "Outline"]
    object: Row | FieldObstacle


class field_planner:

    def __init__(self, system: 'System', leaflet: leaflet_map) -> None:
        self.log = logging.getLogger("field_friend.field_planner")
        self.field_provider = system.field_provider
        self.odometer = system.odometer
        self.gnss = system.gnss
        self.cultivatable_crops = system.plant_locator.crop_category_names
        self.leaflet_map = leaflet
        self.tab: TabType = "Plants"
        self.active_object: ActiveObject | None = None
        self.active_field: Field | None = None
        self.restore_actives()

        with ui.dialog() as self.clear_field_dialog, ui.card():
            ui.label('Are you sure you want to delete all fields?')
            with ui.row():
                ui.button('Cancel', on_click=self.clear_field_dialog.close)
                ui.button('Yes, delete', on_click=self.clear_fields, color='warning')

        with ui.row().classes("w-full").style("height: 100%; max-height:100%; width: 100%;"):
            with ui.card().style("width: 40%; max-width: 40%; max-height: 100%; height: 100%;"):
                with ui.row():
                    ui.button("Upload Field", on_click=lambda field_provider=self.field_provider: geodata_importer(field_provider)) \
                        .tooltip("Upload a file with field boundaries. Supported file formats: KML, XML and Shape").classes("ml-auto").style("display: block; margin-top:auto; margin-bottom: auto;")
                    ui.button("Add field", on_click=self.field_provider.create_field).tooltip("Add a new field") \
                        .classes("ml-auto").style("display: block; margin-top:auto; margin-bottom: auto;")
                    ui.button("Field Wizard", on_click=lambda: FieldCreator(system)).tooltip("Build a field with rows in a few simple steps") \
                        .classes("ml-auto").style("display: block; margin-top:auto; margin-bottom: auto;")
                with ui.row().style("width: 100%;"):
                    self.show_field_table()
                with ui.row():
                    ui.button('Clear fields', on_click=self.clear_field_dialog.open).props("outline color=warning") \
                        .tooltip("Delete all fields").classes("ml-auto").style("display: block; margin-top:auto; margin-bottom: auto;")
                    ui.button("Update reference", on_click=self.field_provider.update_reference).props("outline color=warning") \
                        .tooltip("Set current position as geo reference and restart the system").classes("ml-auto").style("display: block; margin-top:auto; margin-bottom: auto;")
            self.show_field_settings()
            self.show_object_settings()
            self.field_provider.FIELDS_CHANGED.register_ui(self.refresh_ui)

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
        with ui.dialog() as self.delete_active_field_dialog, ui.card():
            ui.label('Are you sure you want to delete this field?')
            with ui.row():
                ui.button('Cancel', on_click=self.delete_active_field_dialog.close)
                ui.button('Yes, delete', on_click=lambda field=self.active_field: self.delete_field(
                    field), color='warning')
        with ui.card().classes('col-grow').style("max-height: 100%; height: 100%;"):
            if self.active_field is None:
                with ui.column().style("display: block; margin: auto;"):
                    ui.icon("polyline").props("size=lg color=primary").style("display: block; margin: auto;")
                    ui.label("select a field").style("display: block; margin: auto; color: #6E93D6;")
            else:
                with ui.row().style("width: 100%"):
                    ui.icon("polyline").props("size=lg color=primary") \
                        .style("display:block; margin-top:auto; margin-bottom: auto;")
                    ui.input(value=f"{self.active_field.name}") \
                        .on("blur", self.field_provider.request_backup) \
                        .bind_value(self.active_field, "name") \
                        .classes("w-32")
                    ui.button(on_click=self.delete_active_field_dialog.open) \
                        .props("icon=delete color=warning fab-mini flat") \
                        .classes("ml-auto").style("display: block; margin-top:auto; margin-bottom: auto;") \
                        .tooltip("Delete field")
                with ui.tabs().style("width: 100%;") as self.tabs:
                    ui.tab("Plants", "Plants")
                    ui.tab("Outline", "Outline")
                    ui.tab("Obstacles", "Obstacles")
                    ui.tab("Rows", "Rows")
                with ui.tab_panels(self.tabs, value=f"{self.tab}", on_change=self.set_tab).style("width: 100%;") as self.panels:
                    with ui.tab_panel("Plants").style("width: 100%;"):
                        ui.select(self.cultivatable_crops, label="Cultivated Crop", on_change=self.field_provider.request_backup) \
                            .classes("w-40").props('clearable') \
                            .bind_value(self.active_field, "crop") \
                            .tooltip('Set the cultivated crop which should be kept safe')
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
                                ui.radio(radio_el, on_change=lambda event: self._set_active_object(
                                    event.value, self.tab), value=self.active_object["object"].id)
                            else:
                                ui.radio(
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
                                ui.radio(radio_el, on_change=lambda event:
                                         self._set_active_object(event.value, self.tab), value=self.active_object["object"].id)
                            else:
                                ui.radio(
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
                    with ui.dialog() as self.delete_obstacle_dialog, ui.card():
                        ui.label('Are you sure you want to delete this obstacle?')
                        with ui.row():
                            ui.button('Cancel', on_click=self.delete_obstacle_dialog.close)
                            ui.button('Yes, delete', on_click=lambda field=self.active_field,
                                      obstacle=self.active_object['object']: self.delete_obstacle(field, obstacle), color='warning')
                    with ui.row().style("width: 100%;"):
                        ui.icon("dangerous").props("size=sm color=primary") \
                            .style("display:block; margin-top:auto; margin-bottom: auto;")
                        ui.input("Obstacle name", value=f'{self.active_object["object"].name}',) \
                            .on("blur", self.field_provider.invalidate).bind_value(self.active_object["object"], "name").classes("w-32")
                        ui.button(on_click=self.delete_obstacle_dialog.open) \
                            .props("icon=delete color=warning fab-mini flat").classes("ml-auto").style("display:block; margin-top:auto; margin-bottom: auto;").tooltip("Delete obstacle")
                    with ui.column().style("display: block; overflow: auto; width: 100%"):
                        assert self.active_field is not None
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
                                      .add_obstacle_point(field, obstacle)).props("icon=add color=primary fab-mini flat")
                            ui.button("", on_click=lambda obstacle=self.active_object["object"]: self.field_provider.remove_obstacle_point(obstacle)) \
                                .props("icon=remove color=warning fab-mini flat")
                elif self.tab == "Rows":
                    with ui.dialog() as self.delete_row_dialog, ui.card():
                        ui.label('Are you sure you want to delete this row?')
                        with ui.row():
                            ui.button('Cancel', on_click=self.delete_row_dialog.close)
                            ui.button('Yes, delete', on_click=lambda field=self.active_field,
                                      row=self.active_object['object']: self.delete_row(field, row), color='warning')
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
                        ui.button(on_click=self.delete_row_dialog.open) \
                            .props("icon=delete color=warning fab-mini flat").classes("ml-auto").style("display:block; margin-top:auto; margin-bottom: auto;").tooltip("Delete Row")
                    with ui.column().style("display: block; overflow: auto; width: 100%"):
                        assert self.active_field is not None
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
                            ui.button("", on_click=lambda field=self.active_field, row=self.active_object["object"]: self.field_provider.add_row_point(field, row)) \
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
                if self.active_object and self.active_object.get("object") is not None:
                    if not (self.active_object.get("object") in self.active_field.obstacles or self.active_object.get("object") in self.active_field.rows):
                        self.active_object = None
                    else:
                        self.tab = self.active_object["object_type"]
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
            app.storage.user['active_field'] = self.active_field.id
        else:
            app.storage.user['active_field'] = None
        self.show_field_settings.refresh()
        self.show_object_settings.refresh()

    def _set_active_object(self, object_id: Optional[str] = None, object_type: TabType | None = None) -> None:
        if (self.active_field is not None and object_id is not None and object_type is not None):
            if object_type == "Obstacles":
                self.active_object = next(
                    ({"object_type": object_type, "object": obj}
                     for obj in self.active_field.obstacles if obj.id == object_id),
                    None
                )
                app.storage.user['active_object'] = {
                    "object_type": "Obstacles", "object": self.active_object["object"].id}
            elif object_type == "Rows":
                self.active_object = next(
                    ({"object_type": object_type, "object": obj}
                     for obj in self.active_field.rows if obj.id == object_id),
                    None
                )
                app.storage.user['active_object'] = {
                    "object_type": "Rows", "object": self.active_object["object"].id}
            else:
                self.active_object = None
                app.storage.user['active_object'] = None
        else:
            self.active_object = None
            app.storage.user['active_object'] = None
        self.show_object_settings.refresh()

    def delete_field(self, field: Field) -> None:
        self.delete_active_field_dialog.close()
        self.field_provider.remove_field(field)

    def clear_fields(self) -> None:
        self.clear_field_dialog.close()
        self.field_provider.clear_fields()

    def delete_obstacle(self, field: Field, obstacle: FieldObstacle) -> None:
        self.delete_obstacle_dialog.close()
        self.field_provider.remove_obstacle(field, obstacle)

    def delete_row(self, field: Field, row: Row) -> None:
        self.delete_row_dialog.close()
        self.field_provider.remove_row(field, row)

    def restore_actives(self) -> None:
        stored_field = app.storage.user.get('active_field', None)
        if stored_field is not None:
            self.active_field = self.field_provider.get_field(stored_field)
        stored_object = app.storage.user.get('active_object', None)
        if self.active_field is not None and stored_object is not None:
            if stored_object["object_type"] == "Obstacles":
                self.active_object = next(({"object_type": stored_object["object_type"], "object": obj}
                                           for obj in self.active_field.obstacles if obj.id == stored_object["object"]), None)
            elif stored_object["object_type"] == "Rows":
                self.active_object = next(({"object_type": stored_object["object_type"], "object": obj}
                                           for obj in self.active_field.rows if obj.id == stored_object["object"]), None)
            self.refresh_ui()
