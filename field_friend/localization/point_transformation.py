import numpy as np
from geographiclib.geodesic import Geodesic
from rosys.geometry import Point

from .geo_point import GeoPoint


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


def wgs84_to_cartesian(reference: GeoPoint, point: GeoPoint) -> Point:
    """
    Converts a GeoPoint from WGS84 coordinates to Cartesian point relative to a reference GeoPoint.
    Args:
        reference (GeoPoint): The reference point in WGS84 coordinates (latitude and longitude).
        point (GeoPoint): The point to be converted in WGS84 coordinates (latitude and longitude).
    Returns:
        Point: The Cartesian coordinates of the point relative to the reference point.
    """
    print(f'{reference.lat} and {reference.long} but point is {point.lat} and {point.long}')
    r = Geodesic.WGS84.Inverse(reference.lat, reference.long, point.lat, point.long)
    s = r['s12']
    a = np.deg2rad(r['azi1'])  # Azimuth in radians (bearing from reference to point)
    a_cartesian = np.pi/2 - a
    x = s * np.cos(a_cartesian)
    y = s * np.sin(a_cartesian)
    cartesian_coords = Point(x=x, y=y)
    return cartesian_coords


def cartesian_to_wgs84(reference: GeoPoint, point: Point) -> GeoPoint:
    """
    Converts a Cartesian Point to WGS84 geographic coordinates Geopoint.

    Args:
        reference (GeoPoint): The reference point in WGS84 coordinates.
        point (Point): The point in Cartesian coordinates to be transformed.

    Returns:
        GeoPoint: The transformed point in WGS84 coordinates.
    """
    r = Geodesic.WGS84.Direct(reference.lat, reference.long, 90.0, point.x)
    r = Geodesic.WGS84.Direct(r['lat2'], r['lon2'], 0.0, point.y)
    wgs84_coords = GeoPoint.from_list([r['lat2'], r['lon2']])
    return wgs84_coords
