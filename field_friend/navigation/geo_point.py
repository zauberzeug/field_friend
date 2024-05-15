
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
import rosys
import shapely
from geographiclib.geodesic import Geodesic


@dataclass(slots=True, kw_only=True)
class GeoPoint:
    lat: float
    long: float

    @staticmethod
    def from_list(point: list[float]) -> GeoPoint:
        return GeoPoint(lat=point[0], long=point[1])

    @property
    def tuple(self) -> tuple[float, float]:
        return (self.lat, self.long)

    def distance(self, other: GeoPoint) -> float:
        geodetic_measurement = Geodesic.WGS84.Inverse(self.lat, self.long, other.lat, other.long)
        return geodetic_measurement['s12']

    def cartesian(self, reference: GeoPoint) -> rosys.geometry.Point:
        r = Geodesic.WGS84.Inverse(reference.lat, reference.long, self.lat, self.long)
        s = r['s12']
        a = -np.deg2rad(r['azi1'])
        x = s * np.cos(a)
        y = s * np.sin(a)
        return rosys.geometry.Point(x=x, y=y)

    def shifted(self, point: rosys.geometry.Point) -> GeoPoint:
        r = Geodesic.WGS84.Direct(self.lat, self.long, 90.0, point.x)
        r = Geodesic.WGS84.Direct(r['lat2'], r['lon2'], 0.0, point.y)
        return GeoPoint(lat=r['lat2'], long=r['lon2'])


@dataclass(slots=True, kw_only=True)
class GeoPointCollection:
    id: str
    name: str
    points: list[GeoPoint] = field(default_factory=list)

    def cartesian(self, reference: GeoPoint) -> list[rosys.geometry.Point]:
        cartesian_points = []
        for point in self.points:
            cartesian_points.append(point.cartesian(reference))
        return cartesian_points

    @property
    def points_as_tuples(self) -> list[tuple[float, float]]:
        return [p.tuple for p in self.points]

    @property
    def shapely_polygon(self) -> shapely.geometry.Polygon:
        return shapely.geometry.Polygon([p.tuple for p in self.points])

    @property
    def shapely_line(self) -> shapely.geometry.LineString:
        return shapely.geometry.LineString([p.tuple for p in self.points])
