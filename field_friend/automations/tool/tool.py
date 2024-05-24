import abc


class Tool(abc.ABC):

    def __init__(self, name: str) -> None:
        self.name = name

    @abc.abstractmethod
    async def prepare(self) -> bool:
        """Prepare the tool once at the beginning (for reference points, etc.); 

        return False if preparation failed."""

    @abc.abstractmethod
    async def activate(self):
        """Activate the tool (for example to start weeding in a new row)"""

    @abc.abstractmethod
    async def deactivate(self):
        """Deactivate the tool (for example to stop weeding at the row's end)"""

    @abc.abstractmethod
    async def observe(self) -> None:
        """Perform observation in a loop; exiting will stop the robot and execute the on_focus method."""

    @abc.abstractmethod
    async def on_focus(self) -> None:
        """Called after robot has stopped via observation to focus on a specific point on the ground"""

    @abc.abstractmethod
    def settings_ui(self):
        """Create UI for settings and configuration."""

    @abc.abstractmethod
    def reset_kpis(self):
        """Reset KPIs for the tool."""
