from dataclasses import dataclass
from datetime import datetime, timedelta
from random import randint
from typing import Any

import rosys
from rosys.analysis import Day, KpiLogger, Month, date_to_str


@dataclass(slots=True, kw_only=True)
class KPIs:
    distance: int = 0
    time_working: int = 0
    time_charging: int = 0

    weeds_detected: int = 0
    crops_detected: int = 0
    punches: int = 0

    bumps: int = 0
    e_stop_triggered:  int = 0
    gnss_failed: int = 0

    automation_started: int = 0
    automation_paused: int = 0
    automation_stopped: int = 0
    automation_failed: int = 0
    automation_completed: int = 0


class KpiProvider(KpiLogger):
    def __init__(self) -> None:
        super().__init__()
        self.all_time_kpis: KPIs = KPIs()
        self.needs_backup: bool = False

    def backup_to_dict(self) -> dict[str, Any]:
        logger_backup = super().backup_to_dict()
        return {'all_time_kpis': rosys.persistence.to_dict(self.all_time_kpis),
                'days': logger_backup['days'],
                'months': logger_backup['months']}

    def restore_from_dict(self, data: dict[str, Any]) -> None:
        super().restore_from_dict(data)
        rosys.persistence.replace_dataclass(self.all_time_kpis, data.get('all_time_kpis', KPIs()))

    def invalidate(self) -> None:
        self.request_backup()

    def increment_all_time_kpi(self, indicator: str, increment: float = 1) -> None:
        self.increment(indicator)
        if getattr(self.all_time_kpis, indicator) is None:
            new_value = increment
        else:
            new_value = getattr(self.all_time_kpis, indicator)+increment
        setattr(self.all_time_kpis, indicator, new_value)
        self.invalidate()

    def get_time_working_kpi(self) -> str:
        total_seconds = int(self.all_time_kpis.time_working)
        hours = total_seconds // 3600
        minutes = (total_seconds % 3600) // 60
        seconds = total_seconds % 60
        return f'{hours:02}:{minutes:02}:{seconds:02}'

    def simulate_kpis(self) -> None:
        simulated_values = {
            'time_working': lambda: randint(0, 3600),
            'time_charging': lambda: randint(0, 3600),
            'distance': lambda: randint(0, 1000),

            'bumps': lambda: randint(0, 5),
            'e_stop_triggered': lambda: randint(0, 10),
            'gnss_failed': lambda: randint(0, 20),

            'crops_detected': lambda: randint(0, 100),
            'weeds_detected': lambda: randint(0, 500),
            'punches': lambda: randint(0, 200),

            'automation_started': lambda: randint(0, 2),
            'automation_paused': lambda: randint(0, 2),
            'automation_stopped': lambda: randint(0, 2),
            'automation_failed': lambda: randint(0, 2),
            'automation_completed': lambda: randint(0, 2),
        }
        self.days = [
            Day(
                date=date_to_str(datetime.today().date() - timedelta(days=i)),
                incidents={key: simulate_value() for key, simulate_value in simulated_values.items()},
            )
            for i in range(1, 7 * 12)
        ][::-1]
        self.months = [
            Month(
                date=date_to_str(datetime.today().date() - timedelta(weeks=i*4)),
                incidents={key: simulate_value() * 7 * 4 for key, simulate_value in simulated_values.items()},
            )
            for i in range(4, 10)
        ][::-1]
