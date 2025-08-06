from typing import Any

import rosys
from rosys.analysis import track
from rosys.geometry import Point


class Implement(rosys.persistence.Persistable):

    def __init__(self, name: str = 'None') -> None:
        super().__init__()
        self.name = name
        self.is_active = False

    async def prepare(self) -> bool:
        """Prepare the implement once at the beginning (for reference points, etc.);

        return False if preparation failed."""
        return True

    @track
    async def finish(self) -> None:
        """Finish the implement once at the end"""
        return None

    @track
    async def activate(self):
        """Activate the implement (for example to start weeding in a new row)"""
        self.is_active = True

    @track
    async def deactivate(self):
        """Deactivate the implement (for example to stop weeding at the row's end)"""
        self.is_active = False

    @track
    async def get_target(self) -> Point | None:
        """Return the target position to drive to."""
        return None

    @track
    async def start_workflow(self) -> None:
        """Called after robot has stopped via observation to perform it's workflow on a specific point on the ground

        Returns True if the robot can drive forward, if the implement whishes to stay at the current location, return False
        """

    @track
    async def stop_workflow(self) -> None:
        """Called after workflow has been performed to stop the workflow"""
        return None

    def backup_to_dict(self) -> dict[str, Any]:
        return {}

    def restore_from_dict(self, data: dict[str, Any]) -> None:
        pass

    def settings_ui(self) -> None:
        """Create UI for settings and configuration."""
        return None
