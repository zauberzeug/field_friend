from typing import List

import numpy as np
from geographiclib.geodesic import Geodesic


def wgs84_to_cartesian(reference, point) -> List[float]:
    r = Geodesic.WGS84.Inverse(reference[0], reference[1], point[0], point[1])
    s = r['s12']
    a = -np.deg2rad(r['azi1'])
    x = s * np.cos(a)
    y = s * np.sin(a)
    cartesian_coords = [x, y]
    return cartesian_coords


def cartesian_to_wgs84(reference, point) -> List[float]:
    r = Geodesic.WGS84.Direct(reference[0], reference[1], 90.0, point[0])
    r = Geodesic.WGS84.Direct(r['lat2'], r['lon2'], 0.0, point[1])
    wgs84_coords = [r['lat2'], r['lon2']]
    return wgs84_coords


def get_new_position(reference, distance, yaw):
    """
    Calculate a new position given a reference point, distance, and yaw (direction in radians).

    Parameters:
    - reference: Tuple containing the latitude and longitude of the reference point (lat, lon).
    - distance: Distance to move from the reference point in meters.
    - yaw: Direction of movement in radians from the north.

    Returns:
    - Tuple containing the latitude and longitude of the new position (lat, lon).
    """
    azimuth_deg = np.degrees(-yaw)
    result = Geodesic.WGS84.Direct(reference[0], reference[1], azimuth_deg, distance)
    new_position = [result['lat2'], result['lon2']]
    return new_position
