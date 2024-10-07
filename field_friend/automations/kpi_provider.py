from dataclasses import dataclass, field
from typing import Any

import rosys
from rosys.analysis import KpiLogger

from .plant_provider import PlantProvider


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
    def __init__(self, plant_provider: PlantProvider) -> None:
        super().__init__()
        self.plant_provider = plant_provider

        self.plant_provider.ADDED_NEW_CROP.register(lambda: self.increment_weeding_kpi('crops_detected'))
        self.plant_provider.ADDED_NEW_WEED.register(lambda: self.increment_weeding_kpi('weeds_detected'))

        self.current_weeding_kpis: Weeding_KPIs = Weeding_KPIs()
        self.WEEDING_KPIS_UPDATED = rosys.event.Event()
        """one of the KPIs of the running weeding automation has been updated."""

        self.all_time_kpis: KPIs = KPIs()

        self.current_mowing_kpis: Mowing_KPIs = Mowing_KPIs()
        self.MOWING_KPIS_UPDATED = rosys.event.Event()
        """one of the KPIs of the running mowing automation has been updated."""

        self.needs_backup: bool = False

    def backup(self) -> dict:
        logger_backup = super().backup()
        return {'current_weeding_kpis': rosys.persistence.to_dict(self.current_weeding_kpis),
                'current_mowing_kpis': rosys.persistence.to_dict(self.current_mowing_kpis),
                'all_time_kpis': rosys.persistence.to_dict(self.all_time_kpis),
                'days': logger_backup['days'],
                'months': logger_backup['months']}

    def restore(self, data: dict[str, Any]) -> None:
        super().restore(data)
        rosys.persistence.replace_dataclass(self.current_weeding_kpis, data.get('current_weeding_kpis', Weeding_KPIs()))
        rosys.persistence.replace_dataclass(self.current_mowing_kpis, data.get('current_mowing_kpis', Mowing_KPIs()))
        rosys.persistence.replace_dataclass(self.all_time_kpis, data.get('all_time_kpis', KPIs()))

    def invalidate(self) -> None:
        self.request_backup()
        self.WEEDING_KPIS_UPDATED.emit()
        self.MOWING_KPIS_UPDATED.emit()

    def increment_all_time_kpi(self, indicator: str) -> None:
        self.increment(indicator)
        if getattr(self.all_time_kpis, indicator) is None:
            new_value = 1
        else:
            new_value = getattr(self.all_time_kpis, indicator)+1
        setattr(self.all_time_kpis, indicator, new_value)
        self.invalidate()
        return
    
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
