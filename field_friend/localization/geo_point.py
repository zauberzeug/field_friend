
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
import rosys
import shapely
from geographiclib.geodesic import Geodesic

from .. import localization


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

    def cartesian(self) -> rosys.geometry.Point:
        """Calculate the cartesian coordinates of this point relative to the reference point.

        This method uses the geodesic distance and azimuth angle between the reference point and
        the current point to calculate the Cartesian coordinates.
        The azimuth angle is measured clockwise from the north direction, and the resulting Cartesian coordinates have the x-axis to north and y-axis to west.

        Returns:
        rosys.geometry.Point: A Point object representing the Cartesian coordinates (x, y)
                            relative to the reference point, where:
                            - x is the eastward distance in meters,
                            - y is the northward distance in meters.
        """
        r = Geodesic.WGS84.Inverse(localization.reference.lat, localization.reference.long, self.lat, self.long)
        s = r['s12']
        a = -np.deg2rad(r['azi1'])
        x = s * np.cos(a)
        y = s * np.sin(a)
        return rosys.geometry.Point(x=x, y=y)

    def shifted(self, point: rosys.geometry.Point) -> GeoPoint:
        """Shift by the given Cartesian coordinates (x, y) relative to the current point.

        Note: the RoSys coordinate system maps x-axis to north and y-axis to west.

        Parameters:
        point (rosys.geometry.Point): A Point object representing the shift in meters, where:
                                    - x is the eastward shift in meters,
                                    - y is the northward shift in meters.

        Returns:
        GeoPoint: A new GeoPoint object representing the shifted geographic coordinates.
        """
        north_shift = Geodesic.WGS84.Direct(self.lat, self.long, 0.0, point.x)
        west_shift = Geodesic.WGS84.Direct(north_shift['lat2'], north_shift['lon2'], 270.0, point.y)
        return GeoPoint(lat=west_shift['lat2'], long=west_shift['lon2'])

    def __str__(self) -> str:
        return f'GeoPoint({round(self.lat, 5)}, {round(self.long, 5)})'


@dataclass(slots=True, kw_only=True)
class GeoPointCollection:
    id: str
    name: str
    points: list[GeoPoint] = field(default_factory=list)

    def cartesian(self) -> list[rosys.geometry.Point]:
        cartesian_points = []
        for point in self.points:
            cartesian_points.append(point.cartesian())
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


def get_new_position(reference: GeoPoint, distance: float, yaw: float) -> GeoPoint:
    """Calculate a new position given a reference point, distance, and yaw (direction in radians).

    Parameters:
    - reference: Tuple containing the latitude and longitude of the reference point (lat, lon).
    - distance: Distance to move from the reference point in meters.
    - yaw: Direction of movement in radians from the north.

    Returns:
    - Tuple containing the latitude and longitude of the new position (lat, lon).
    """
    azimuth_deg = np.degrees(-yaw)
    result = Geodesic.WGS84.Direct(reference.lat, reference.long, azimuth_deg, distance)
    new_position = [result['lat2'], result['lon2']]
    return GeoPoint.from_list(new_position)
