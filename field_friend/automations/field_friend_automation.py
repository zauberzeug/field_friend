import abc


class FieldFriendAutomation(abc.ABC):

    def __init__(self, name: str) -> None:
        self.name = name

    @abc.abstractmethod
    async def start(self):
        """Start the automation"""

    @abc.abstractmethod
    def settings_ui(self):
        """Create UI for settings and configuration."""
