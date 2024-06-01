import abc


class Implement(abc.ABC):

    def __init__(self, name: str) -> None:
        self.name = name

    @abc.abstractmethod
    async def prepare(self) -> bool:
        """Prepare the implement once at the beginning (for reference points, etc.); 

        return False if preparation failed."""

    @abc.abstractmethod
    async def finish(self) -> None:
        """Finish the implement once at the end (for example to store KPIs)"""

    @abc.abstractmethod
    async def activate(self):
        """Activate the implement (for example to start weeding in a new row)"""

    @abc.abstractmethod
    async def deactivate(self):
        """Deactivate the implement (for example to stop weeding at the row's end)"""

    @abc.abstractmethod
    async def observe(self) -> None:
        """Run a custom observation in a loop; exiting will stop the robot and execute the perform_workflow method."""

    @abc.abstractmethod
    async def start_workflow(self) -> None:
        """Called after robot has stopped via observation to perform it's workflow on a specific point on the ground"""

    async def stop_workflow(self) -> None:
        """Called after workflow has been performed to stop the workflow"""

    @abc.abstractmethod
    def settings_ui(self):
        """Create UI for settings and configuration."""

    @abc.abstractmethod
    def reset_kpis(self):
        """Reset KPIs for the implement."""
