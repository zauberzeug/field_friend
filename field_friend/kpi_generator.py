from datetime import datetime, timedelta
from random import randint
from .automations import KpiProvider
from rosys.analysis import Day, Month, date_to_str


def generate_kpis(kpi_provider: KpiProvider) -> None:
    kpi_provider.days = [
        Day(
            date=date_to_str(datetime.today().date() - timedelta(days=i)),
            incidents={
                'bumps': randint(0, 5),
                'low_battery': randint(0, 3),
                'can_failure': randint(0, 2),
                'automation_stopped': randint(0, 2),
                'e_stop_triggered': randint(0, 10),
                'soft_e_stop_triggered': randint(0, 10),
                'imu_rolling_detected': randint(0, 2),
                'gnss_connection_lost': randint(0, 20),
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
    kpi_provider.days[5].incidents['mowing_completed'] = 0
    kpi_provider.days[1].incidents.clear()

    kpi_provider.months = [
        Month(
            date=date_to_str(datetime.today().date() - timedelta(weeks=i*4)),
            incidents={
                'bumps': randint(0, 5) * 7 * 4,
                'low_battery': randint(0, 3) * 7 * 4,
                'can_failure': randint(0, 2) * 7 * 4,
                'automation_stopped': randint(0, 2) * 7 * 4,
                'e_stop_triggered': randint(0, 10) * 7 * 4,
                'soft_e_stop_triggered': randint(0, 10) * 7 * 4,
                'imu_rolling_detected': randint(0, 2) * 7 * 4,
                'gnss_connection_lost': randint(0, 20) * 7 * 4,
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
