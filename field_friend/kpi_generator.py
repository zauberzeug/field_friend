from datetime import datetime, timedelta
from random import randint

from rosys.analysis import Day, KpiLogger, Month, date_to_str


def generate_kpis(kpi_logger: KpiLogger) -> None:
    kpi_logger.days = [
        Day(
            date=date_to_str(datetime.today().date() - timedelta(days=i)),
            incidents={
                'bumps': randint(0, 5),
                'low_battery': randint(0, 3),
                'can_failure': randint(0, 2),
                'automation_stopped': randint(0, 2),
                'coin_collecting_completed': randint(0, 10),
                'rows_weeded': randint(0, 100),
                'weeding_completed': randint(0, 10),
                'mowing_completed': randint(0, 10),
                'crops_detected': randint(0, 100),
                'weeds_detected': randint(0, 500),
                'punches': randint(0, 200),
            },
        )
        for i in range(7 * 12)
    ][::-1]
    kpi_logger.days[5].incidents['mowing_completed'] = 0
    kpi_logger.days[1].incidents.clear()

    kpi_logger.months = [
        Month(
            date=date_to_str(datetime.today().date() - timedelta(weeks=i*4)),
            incidents={
                'bumps': randint(0, 5) * 7 * 4,
                'low_battery': randint(0, 3) * 7 * 4,
                'can_failure': randint(0, 2) * 7 * 4,
                'automation_stopped': randint(0, 2) * 7 * 4,
                'coin_collecting_completed': randint(0, 10) * 7 * 4,
                'rows_weeded': randint(0, 100) * 7 * 4,
                'weeding_completed': randint(0, 10) * 7 * 4,
                'mowing_completed': randint(0, 10) * 7 * 4,
                'crops_detected': randint(0, 100) * 7 * 4,
                'weeds_detected': randint(0, 500) * 7 * 4,
                'punches': randint(0, 200) * 7 * 4,
            },
        )
        for i in range(4, 10)
    ][::-1]
