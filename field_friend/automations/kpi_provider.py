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
class KpiProvider(KpiLogger):
    def __init__(self, plant_provider: PlantProvider) -> None:
        super().__init__()
        self.all_time_kpis: KPIs = KPIs()
        self.needs_backup: bool = False

    def backup(self) -> dict:
        logger_backup = super().backup()
        return {'all_time_kpis': rosys.persistence.to_dict(self.all_time_kpis),
                'days': logger_backup['days'],
                'months': logger_backup['months']}

    def restore(self, data: dict[str, Any]) -> None:
        super().restore(data)
        rosys.persistence.replace_dataclass(self.all_time_kpis, data.get('all_time_kpis', KPIs()))

    def invalidate(self) -> None:
        self.request_backup()

    def increment_all_time_kpi(self, indicator: str,increment:float=1) -> None:
        self.increment(indicator)
        if getattr(self.all_time_kpis, indicator) is None:
            new_value = increment
        else:
            new_value = getattr(self.all_time_kpis, indicator)+increment
        setattr(self.all_time_kpis, indicator, new_value)
        self.invalidate()
        return

    def get_time_kpi(self) -> str:
        total_seconds = int(self.all_time_kpis.time)
        hours = total_seconds // 3600
        minutes = (total_seconds % 3600) // 60
        seconds = total_seconds % 60
        return f'{hours:02}:{minutes:02}:{seconds:02}'
