import abc


class Implement(abc.ABC):

    def __init__(self, name: str) -> None:
        self.name = name
        self.is_active = False

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

    async def observe(self) -> None:
        """Run a custom observation in a loop; exiting will stop the robot and execute the perform_workflow method."""

    async def start_workflow(self) -> None:
        """Called after robot has stopped via observation to perform it's workflow on a specific point on the ground"""

    async def stop_workflow(self) -> None:
        """Called after workflow has been performed to stop the workflow"""

    def settings_ui(self):
        """Create UI for settings and configuration."""

    def reset_kpis(self):
        """Reset KPIs for the implement."""
