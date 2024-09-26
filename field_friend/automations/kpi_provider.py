from dataclasses import dataclass
from datetime import timedelta
from typing import TYPE_CHECKING, Any

import rosys
from nicegui import ui
from rosys.analysis import KpiLogger

if TYPE_CHECKING:
    from field_friend.system import System


@dataclass(slots=True, kw_only=True)
class KPIs:
    distance: int = 0
    time: int = 0
    bumps: int = 0
    low_battery: int = 0
    can_failure: int = 0
    automation_stopped: int = 0
    e_stop_triggered:  int = 0
    soft_e_stop_triggered:  int = 0
    imu_rolling_detected: int = 0
    gnss_connection_lost: int = 0


@dataclass(slots=True, kw_only=True)
class Weeding_KPIs(KPIs):
    rows_weeded: int = 0
    crops_detected: int = 0
    weeds_detected: int = 0
    weeds_removed: int = 0
    punches: int = 0
    chops: int = 0
    weeding_completed: bool = False


@dataclass(slots=True, kw_only=True)
class Mowing_KPIs(KPIs):
    mowing_completed: bool = False


class KpiProvider(KpiLogger):
    def __init__(self, system: 'System') -> None:
        super().__init__()
        self.system = system

        system.plant_provider.ADDED_NEW_CROP.register(lambda: self.increment_weeding_kpi('crops_detected'))
        system.plant_provider.ADDED_NEW_WEED.register(lambda: self.increment_weeding_kpi('weeds_detected'))

        self.current_weeding_kpis: Weeding_KPIs = Weeding_KPIs()
        self.WEEDING_KPIS_UPDATED = rosys.event.Event()
        """one of the KPIs of the running weeding automation has been updated."""

        self.current_mowing_kpis: Mowing_KPIs = Mowing_KPIs()
        self.MOWING_KPIS_UPDATED = rosys.event.Event()
        """one of the KPIs of the running mowing automation has been updated."""

        self.needs_backup: bool = False

    def backup(self) -> dict:
        logger_backup = super().backup()
        return {'current_weeding_kpis': rosys.persistence.to_dict(self.current_weeding_kpis),
                'current_mowing_kpis': rosys.persistence.to_dict(self.current_mowing_kpis),
                'days': logger_backup['days'],
                'months': logger_backup['months']}

    def restore(self, data: dict[str, Any]) -> None:
        super().restore(data)
        rosys.persistence.replace_dataclass(self.current_weeding_kpis, data.get('current_weeding_kpis', Weeding_KPIs()))
        rosys.persistence.replace_dataclass(self.current_mowing_kpis, data.get('current_mowing_kpis', Mowing_KPIs()))

    def invalidate(self) -> None:
        self.request_backup()
        self.WEEDING_KPIS_UPDATED.emit()
        self.MOWING_KPIS_UPDATED.emit()

    def increment_weeding_kpi(self, indicator: str) -> None:
        self.increment(indicator)
        if getattr(self.current_weeding_kpis, indicator) is None:
            new_value = 1
        else:
            new_value = getattr(self.current_weeding_kpis, indicator)+1
        setattr(self.current_weeding_kpis, indicator, new_value)
        self.WEEDING_KPIS_UPDATED.emit()
        self.invalidate()
        return

    def increment_mowing_kpi(self, indicator: str) -> None:
        self.increment(indicator)
        new_value = getattr(self.current_mowing_kpis, indicator)+1
        setattr(self.current_mowing_kpis, indicator, new_value)
        self.MOWING_KPIS_UPDATED.emit()
        self.invalidate()

    def clear_weeding_kpis(self) -> None:
        self.current_weeding_kpis = Weeding_KPIs()
        self.WEEDING_KPIS_UPDATED.emit()
        self.invalidate()

    def clear_mowing_kpis(self) -> None:
        self.current_mowing_kpis = Mowing_KPIs()
        self.MOWING_KPIS_UPDATED.emit()
        self.invalidate()

    def developer_ui(self) -> None:
        ui.label('Performance').classes('w-full text-center text-bold')
        ui.separator()
        with ui.row().classes('place-items-center'):
            ui.label('Current Field:').classes('text-bold')
            ui.label()
        with ui.row().classes('place-items-center'):
            ui.label('Worked Area:').classes('text-bold')
            ui.label()
        with ui.row().classes('place-items-center'):
            ui.label('Current Row:').classes('text-bold')
            ui.label()
        with ui.row().classes('place-items-center'):
            ui.label('Time on Field:').classes('text-bold')
            ui.label(f'{timedelta(seconds=self.current_weeding_kpis.time)}')
        with ui.row().classes('place-items-center'):
            ui.label('Distance:').classes('text-bold')
            ui.label(f'{self.current_weeding_kpis.distance:.0f}m')
        with ui.row().classes('place-items-center'):
            ui.label('Processed Rows:').classes('text-bold')
            ui.label()
        with ui.row().classes('place-items-center'):
            ui.label('Crops Detected:').classes('text-bold')
            ui.label()
        with ui.row().classes('place-items-center'):
            ui.label('Weeds Detected:').classes('text-bold')
            ui.label()
        with ui.row().classes('place-items-center'):
            ui.label('Weeds Removed:').classes('text-bold')
            ui.label()
        with ui.row().classes('place-items-center'):
            ui.label('Punches:').classes('text-bold')
            ui.label()
        if self.system.field_friend.implement_name == 'dual_mechanism':
            with ui.row().classes('place-items-center'):
                ui.label('Chops:').classes('text-bold')
                ui.label()

        # OLD CODE, WAS ALREADY COMMENTED OUT IN OTHER FILE
        # current_automation = next(key for key, value in system.implements.items()
        #                           if value == system.automator.default_automation)
        # if current_automation == 'weeding' or current_automation == 'monitoring':
        #     if current_automation == 'weeding':
        #         current_row_label.text = system.weeding.current_row.name if system.weeding.current_row is not None else 'No row'
        #         worked_area_label.text = f'{system.weeding.field.worked_area(system.kpi_provider.current_weeding_kpis.rows_weeded):.2f}m²/{system.weeding.field.area():.2f}m²' if system.weeding.field is not None else 'No field'
        #     elif current_automation == 'monitoring':
        #         current_row_label.text = system.monitoring.current_row.name if system.monitoring.current_row is not None else 'No row'
        #         worked_area_label.text = f'{system.monitoring.field.worked_area(system.kpi_provider.current_weeding_kpis.rows_weeded):.2f}m²/{system.monitoring.field.area():.2f}m²' if system.monitoring.field is not None else 'No field'
        #     kpi_weeds_detected_label.text = system.kpi_provider.current_weeding_kpis.weeds_detected
        #     kpi_crops_detected_label.text = system.kpi_provider.current_weeding_kpis.crops_detected
        #     kpi_weeds_removed_label.text = system.kpi_provider.current_weeding_kpis.weeds_removed
        #     kpi_rows_weeded_label.text = system.kpi_provider.current_weeding_kpis.rows_weeded
        #     if current_automation == 'weeding':
        #         kpi_punches_label.text = system.kpi_provider.current_weeding_kpis.punches
        #         if robot.implement_name == 'dual_mechanism':
        #             kpi_chops_label.text = system.kpi_provider.current_weeding_kpis.chops
