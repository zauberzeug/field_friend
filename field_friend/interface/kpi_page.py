from rosys.analysis import KpiChart, kpi_page

from ..system import System
from .components import create_header


class KpiPage(kpi_page):
    def __init__(self, system: System) -> None:
        super().__init__(system.kpi_provider)
        self.system = system

    def _content(self) -> None:
        create_header(self.system)
        super()._content()

    @property
    def title(self) -> str:
        return 'Key Performance Indicators'

    @property
    def charts(self) -> list[KpiChart]:
        positives = KpiChart(title='Weeding Statistics', indicators={
            'weeds_detected': 'Weeds detected',
            'crops_detected': 'Crops detected',
            'punches': 'Punches'
        })
        # TODO: not working, because only incidents are counted, not the values
        # time = KpiChart(title='Working Time', unit='Seconds', indicators={
        #     'time_working': 'Working',
        #     'time_charging': 'Charging',
        # })
        # distance = KpiChart(title='Distance', unit='Meters', indicators={
        #     'distance': 'Distance driven',
        # })
        negatives = KpiChart(title='Exceptions', indicators={
            'bumps': 'Bumper',
            'e_stop_triggered': 'E-Stop',
            'gnss_failed': 'GNSS failed',
            'low_battery': 'Low battery'
        }, colormap='Reds')
        activities = KpiChart(title='Automation Statistics', indicators={
            'automation_paused': 'Automation paused',
            'automation_stopped': 'Automation stopped',
            'automation_failed': 'Automation failed',
            'automation_completed': 'Automation completed'
        })
        return [positives, negatives, activities]

    @property
    def timespans(self) -> dict[int, str]:
        return {
            7: '7 days',
            28: '4 weeks',
            90: '3 months',
        }
