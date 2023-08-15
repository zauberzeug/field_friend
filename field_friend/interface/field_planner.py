import logging
import uuid
from typing import Optional

import rosys
from nicegui import ui

from ..navigation import Field, FieldObstacle, FieldProvider, Gnss


class field_planner:

    def __init__(self, field_provider: FieldProvider, odometer: rosys.driving.Odometer, gnss: Gnss) -> None:
        self.log = logging.getLogger('field_friend.field_planner')
        self.field_provider = field_provider
        self.odometer = odometer
        self.gnss = gnss
        with ui.card():
            with ui.row():
                ui.button('Add field', on_click=self.add_field).tooltip('Add a new field')
                ui.button('Clear fields', on_click=self.clear_fields).props(
                    'outline color=warning').tooltip('Delete all fields')
            with ui.row():
                self.show_field_settings()

    @ui.refreshable
    def show_field_settings(self) -> None:
        for field in self.field_provider.fields:
            with ui.card().classes('items-stretch'):
                with ui.row().classes('items-center'):
                    ui.icon('fence').props('size=lg color=primary')
                    ui.input(
                        'Field name', value=f'{field.name}', on_change=self.field_provider.invalidate).bind_value(
                        field, 'name').classes('w-32')
                    ui.button(on_click=lambda field=field: self.delete_field(field)) \
                        .props('icon=delete color=warning fab-mini flat').classes('ml-auto').tooltip('Delete field')
                with ui.tabs() as self.tabs:
                    ui.tab('Outline', 'Outline')
                    ui.tab('Obstacles', 'Obstacles')
                with ui.tab_panels(self.tabs, value='Outline') as self.panels:
                    with ui.tab_panel('Outline'):
                        for point in field.outline:
                            with ui.row().classes('items-center'):
                                ui.button(on_click=lambda field=field, point=point: self.add_point(field, point)).props(
                                    'icon=place color=primary fab-mini flat').tooltip('Relocate point').classes('ml-0')
                                ui.number(
                                    'x', value=point.x, format='%.2f', step=0.1,
                                    on_change=self.field_provider.invalidate).bind_value(
                                    point, 'x').classes('w-16')
                                ui.number(
                                    'y', value=point.y, format='%.2f', step=0.1,
                                    on_change=self.field_provider.invalidate).bind_value(
                                    point, 'y').classes('w-16')
                        with ui.row().classes('items-center mt-2'):
                            ui.icon('place').props('size=sm color=grey').classes('ml-2')
                            ui.button('', on_click=lambda field=field: self.add_point(field)) \
                                .props('icon=add color=primary fab-mini flat').tooltip('Add point')
                            ui.button('', on_click=lambda field=field: self.remove_point(field)) \
                                .props('icon=remove color=warning fab-mini flat').tooltip('Remove point')

                    with ui.tab_panel('Obstacles'):
                        for obstacle in field.obstacles:
                            with ui.row().classes('items-center'):
                                ui.icon('block').props('size=sm color=primary')
                                ui.input(
                                    'Obstacle name', value=f'{obstacle.name}').bind_value(
                                    obstacle, 'name').classes('w-32')
                                ui.button(on_click=lambda field=field, obstacle=obstacle: self.remove_obstacle(field, obstacle)).props(
                                    'icon=delete color=warning fab-mini flat').classes('ml-auto').tooltip('Delete obstacle')
                            for point in obstacle.points:
                                with ui.row().classes('items-center'):
                                    ui.button(
                                        on_click=lambda field=field, point=point: self.add_point(field, point)).props(
                                        'icon=place color=primary fab-mini flat').tooltip('Relocate point').classes('ml-6')
                                    ui.number(
                                        'x', value=point.x, format='%.2f', step=0.1,
                                        on_change=self.field_provider.invalidate).bind_value(
                                        point, 'x').classes('w-16')
                                    ui.number(
                                        'y', value=point.y, format='%.2f', step=0.1,
                                        on_change=self.field_provider.invalidate).bind_value(
                                        point, 'y').classes('w-16')
                            with ui.row().classes('items-center mt-2'):
                                ui.icon('place').props('size=sm color=grey').classes('ml-8')
                                ui.button('', on_click=lambda field=field, obstacle=obstacle: self.add_obstacle_point(
                                    field, obstacle)).props('icon=add color=primary fab-mini flat')
                                ui.button('', on_click=lambda obstacle=obstacle: self.remove_obstacle_point(
                                    obstacle)).props('icon=remove color=warning fab-mini flat')

                        with ui.row().classes('items-center mt-3'):
                            ui.icon('block').props('size=sm color=grey')
                            ui.button('ADD OBSTACLE', on_click=lambda field=field: self.add_obstacle(field)) \
                                .props('color=primary outline')

    def add_point(self, field: Field, point: Optional[rosys.geometry.Point] = None) -> None:
        if self.gnss.device != 'simulation':
            if self.gnss.device is None:
                self.log.warning('not creating Point because no GNSS device found')
                rosys.notify('No GNSS device found')
                return
            if self.gnss.record.gps_qual != 4:
                self.log.warning('not creating Point because no RTK fix available')
                rosys.notify('No RTK fix available')
                return
            if field.reference_lat is None or field.reference_lon is None:
                ref_lat, ref_lon = self.gnss.get_reference()
                if ref_lat is None or ref_lon is None:
                    self.log.warning('not creating Point because no reference position available')
                    rosys.notify('No reference position available')
                    return
                field.reference_lat = ref_lat
                field.reference_lon = ref_lon
        if point is not None:
            self.remove_point(field, point)
        point = self.odometer.prediction.point
        field.outline.append(point)
        self.field_provider.invalidate()
        self.show_field_settings.refresh()

    def remove_point(self, field: Field, point: Optional[rosys.geometry.Point] = None) -> None:
        if point is None and field.outline != []:
            point = field.outline[-1]
        field.outline.remove(point)
        self.field_provider.invalidate()
        self.show_field_settings.refresh()
        self.panels.set_value('Outline')

    def add_field(self) -> None:
        field = Field(name=f'{str(uuid.uuid4())}')
        self.field_provider.add_field(field)
        self.show_field_settings.refresh()
        self.panels.set_value('Outline')

    def delete_field(self, field: Field) -> None:
        self.field_provider.remove_field(field)
        self.show_field_settings.refresh()
        self.panels.set_value('Outline')

    def clear_fields(self) -> None:
        self.field_provider.clear_fields()
        self.show_field_settings.refresh()
        self.panels.set_value('Outline')

    def add_obstacle(self, field: Field) -> None:
        obstacle = FieldObstacle(name=f'{str(uuid.uuid4())}')
        self.field_provider.add_obstacle(field, obstacle)
        self.show_field_settings.refresh()
        self.panels.set_value('Obstacles')

    def remove_obstacle(self, field: Field, obstacle: FieldObstacle) -> None:
        self.field_provider.remove_obstacle(field, obstacle)
        self.show_field_settings.refresh()
        self.panels.set_value('Obstacles')

    def add_obstacle_point(
            self, field: Field, obstacle: FieldObstacle, point: Optional[rosys.geometry.Point] = None) -> None:
        if self.gnss.device != 'simulation':
            if self.gnss.device is None:
                self.log.warning('not creating Point because no GNSS device found')
                rosys.notify('No GNSS device found')
                return
            if self.gnss.record.gps_qual != 4:
                self.log.warning('not creating Point because no RTK fix available')
                rosys.notify('No RTK fix available')
                return
            if field.reference_lat is None or field.reference_lon is None:
                ref_lat, ref_lon = self.gnss.get_reference()
                if ref_lat is None or ref_lon is None:
                    self.log.warning('not creating Point because no reference position available')
                    rosys.notify('No reference position available')
                    return
                field.reference_lat = ref_lat
                field.reference_lon = ref_lon
        if point is not None:
            self.remove_obstacle_point(obstacle, point)
        point = self.odometer.prediction.point
        obstacle.points.append(point)
        self.field_provider.invalidate()
        self.show_field_settings.refresh()
        self.panels.set_value('Obstacles')

    def remove_obstacle_point(self, obstacle: FieldObstacle, point: Optional[rosys.geometry.Point] = None) -> None:
        if point is None and obstacle.points != []:
            point = obstacle.points[-1]
        obstacle.points.remove(point)
        self.field_provider.invalidate()
        self.show_field_settings.refresh()
        self.panels.set_value('Obstacles')
