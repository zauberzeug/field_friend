import numpy as np
import rosys
from rosys.geometry import Point

from . import LaserScanner, Scan


class LaserScannerSimulation(LaserScanner):

    NOISE: float = 0.02
    NUM_POINTS: int = 100
    DISTANCE: float = 5.0

    def __init__(self) -> None:
        super().__init__()
        rosys.on_repeat(self.simulate_scan, 0.1)

    async def simulate_scan(self) -> None:
        zero_point = Point(x=0, y=0)
        distances: list[float] = [np.random.uniform(
            self.DISTANCE * (1.0 - self.NOISE), self.DISTANCE * (1.0 + self.NOISE)) for _ in range(self.NUM_POINTS)]
        angles: list[float] = [np.deg2rad(i * 360 / self.NUM_POINTS) for i in range(self.NUM_POINTS)]
        points = [zero_point.polar(distance, angle) for distance, angle in zip(distances, angles)]
        scan = Scan(time=rosys.time(), points=points)
        self.last_scan = scan
        self.NEW_SCAN.emit(scan)
