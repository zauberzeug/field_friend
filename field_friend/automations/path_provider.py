from dataclasses import dataclass, field
from typing import Any

import rosys


@dataclass(slots=True, kw_only=True)
class Path:
    name: str
    path_segments: list[rosys.driving.PathSegment] = field(default_factory=list)
    visualized: bool = False


class PathProvider(rosys.persistence.PersistentModule):
    def __init__(self) -> None:
        super().__init__()
        self.paths: list[Path] = []

        self.PATHS_CHANGED = rosys.event.Event()
        """The dict of paths has changed."""

        self.SHOW_PATH = rosys.event.Event()
        """Show the path in the map."""

        self.needs_backup: bool = False

    def backup(self) -> dict:
        return {'paths': rosys.persistence.to_dict(self.paths)}

    def restore(self, data: dict[str, Any]) -> None:
        rosys.persistence.replace_list(self.paths, Path, data.get('paths', []))

    def invalidate(self) -> None:
        self.request_backup()
        self.PATHS_CHANGED.emit()

    def add_path(self, path: Path) -> None:
        self.paths.append(path)
        self.invalidate()

    def remove_path(self, path: Path) -> None:
        self.paths.remove(path)
        self.invalidate()

    def clear_paths(self) -> None:
        self.paths.clear()
        self.invalidate()
