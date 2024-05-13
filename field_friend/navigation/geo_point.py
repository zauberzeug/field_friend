
from __future__ import annotations

from dataclasses import dataclass


from geographiclib.geodesic import Geodesic


@dataclass(slots=True, kw_only=True)
class GeoPoint:
    lat: float
    long: float

    @property
    def tuple(self) -> tuple[float, float]:
        return (self.lat, self.long)

    def distance(self, other: GeoPoint) -> float:
        geodetic_measurement = Geodesic.WGS84.Inverse(self.lat, self.long, other.lat, other.long)
        return geodetic_measurement['s12']
