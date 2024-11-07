from rosys.analysis import KpiChart, kpi_page

from ..system import System
from .components import create_header


class KpiPage(kpi_page):

    def __init__(self, system: System) -> None:
        super().__init__(system.kpi_provider)
        create_header(system)

    @property
    def language(self) -> str:
        return 'en'

    @property
    def title(self) -> str:
        return 'Key Performance Indicators'

    @property
    def charts(self) -> list[KpiChart]:
        positives = KpiChart(title='Weeding Statistics', indicators={
            'rows_weeded': 'rows weeded',
            'weeds_detected': 'weeds detected',
            'crops_detected': 'crops detected',
            'punches': 'punches'
        })
        negatives = KpiChart(title='Exceptions', indicators={
            'automation_stopped': 'automation stopped',
            'automation_failed': 'automation failed',
            'bumps': 'bumps',
            'wheels_blocking': 'wheels blocking',
            'wheels_slipping': 'wheels slipping',
            'low_battery': 'low battery',
            'can_failure': 'can failure',
            'workflow_problem': 'workflow problem',
            'escape': 'escape',
            'reference_tool_failed': 'reference tool failed',
            'punching_failed': 'punching failed'
        }, colormap='Reds')
        activities = KpiChart(title='Automation Statistics', indicators={
            'mowing_completed': 'mowing completed',
            'weeding_completed': 'weeding completed',
            'coin_collecting_completed': 'coin collecting completed'
        })
        return [positives, negatives, activities]

    @property
    def timespans(self) -> dict[int, str]:
        return {
            7: '7 days',
            28: '4 weeks',
            90: '3 months',
        }
