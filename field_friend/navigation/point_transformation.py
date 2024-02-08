import numpy as np
from geographiclib.geodesic import Geodesic


def wgs84_to_cartesian(reference, point):
    r = Geodesic.WGS84.Inverse(reference[0], reference[1], point[0], point[1])
    s = r['s12']
    a = -np.deg2rad(r['azi1'])
    x = s * np.cos(a)
    y = s * np.sin(a)
    cartesian_coords = [x, y]
    return cartesian_coords


def cartesian_to_wgs84(reference, point):
    r = Geodesic.WGS84.Direct(reference[0], reference[1], 90.0, point[0])
    r = Geodesic.WGS84.Direct(r['lat2'], r['lon2'], 0.0, point[1])
    wgs84_coords = [r['lat2'], r['lon2']]
    return wgs84_coords
