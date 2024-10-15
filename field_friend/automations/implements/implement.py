import abc
from typing import TYPE_CHECKING

import rosys

if TYPE_CHECKING:
    from system import System

class Implement(abc.ABC):

    def __init__(self, name: str, system:'System') -> None:
        self.name = name
        self.system = system
        self.is_active = False
        self.start_time = None
        rosys.on_repeat(self._update_time, 0.1)

    async def prepare(self) -> bool:
        """Prepare the implement once at the beginning (for reference points, etc.); 

        return False if preparation failed."""
        return True

    async def finish(self) -> None:
        """Finish the implement once at the end (for example to store KPIs)"""

    async def activate(self):
        """Activate the implement (for example to start weeding in a new row)"""
        self.is_active = True

    async def deactivate(self):
        """Deactivate the implement (for example to stop weeding at the row's end)"""
        self.is_active = False

    async def get_stretch(self, max_distance: float) -> float:
        """Return the stretch which the implement thinks is safe to drive forward."""
        return 0.02

    async def start_workflow(self) -> None:
        """Called after robot has stopped via observation to perform it's workflow on a specific point on the ground

        Returns True if the robot can drive forward, if the implement whishes to stay at the current location, return False
        """

    async def stop_workflow(self) -> None:
        """Called after workflow has been performed to stop the workflow"""

    def settings_ui(self):
        """Create UI for settings and configuration."""

    def reset_kpis(self):
        """Reset KPIs for the implement."""
    
    def _update_time(self):
        """Update KPIs for time"""
        if not self.is_active:
            return
        if self.start_time is None:
            self.start_time = rosys.time()
        passed_time = rosys.time() - self.start_time
        if passed_time > 1:
            self.system.kpi_provider.increment_all_time_kpi('time')
            self.start_time = rosys.time()
