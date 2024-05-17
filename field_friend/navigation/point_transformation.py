import numpy as np
from geographiclib.geodesic import Geodesic


def get_new_position(reference, distance, yaw):
    """Calculate a new position given a reference point, distance, and yaw (direction in radians).

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
