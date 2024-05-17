
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
        """Calculate the cartesian coordinates of this point relative to a reference point.

        This method uses the geodesic distance and azimuth angle between the reference point and
        the current point to calculate the Cartesian coordinates. The azimuth angle is measured
        clockwise from the north direction, and the resulting Cartesian coordinates follow the 
        standard convention where the x-axis points east and the y-axis points north.

        Parameters:
        reference (GeoPoint): The reference GeoPoint to which the Cartesian coordinates are relative.

        Returns:
        rosys.geometry.Point: A Point object representing the Cartesian coordinates (x, y) 
                            relative to the reference point, where:
                            - x is the eastward distance in meters,
                            - y is the northward distance in meters.
        """
        r = Geodesic.WGS84.Inverse(reference.lat, reference.long, self.lat, self.long)
        s = r['s12']
        a = np.deg2rad(r['azi1'])
        x = s * np.cos(a)
        y = s * np.sin(a)
        return rosys.geometry.Point(x=x, y=y)

    def shifted(self, point: rosys.geometry.Point) -> GeoPoint:
        """Shift by the given Cartesian coordinates (x, y) relative to the current point.

        Parameters:
        point (rosys.geometry.Point): A Point object representing the shift in meters, where:
                                    - x is the eastward shift in meters,
                                    - y is the northward shift in meters.

        Returns:
        GeoPoint: A new GeoPoint object representing the shifted geographic coordinates.
        """
        north_shift = Geodesic.WGS84.Direct(self.lat, self.long, 0.0, point.y)
        east_shift = Geodesic.WGS84.Direct(north_shift['lat2'], north_shift['lon2'], 90.0, point.x)
        return GeoPoint(lat=east_shift['lat2'], long=east_shift['lon2'])


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
