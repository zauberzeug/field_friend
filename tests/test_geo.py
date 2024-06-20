import pytest
from rosys.geometry import Point

from field_friend.localization import GeoPoint


def test_shifting():
    origin = GeoPoint(lat=51.98317071260942, long=7.43411239981148)
    x10 = origin.shifted(Point(x=10, y=0))
    y10 = origin.shifted(Point(x=0, y=10))
    x10y10 = origin.shifted(Point(x=10, y=10))
    assert x10.distance(origin) == pytest.approx(10)
    assert y10.distance(origin) == pytest.approx(10)
    assert x10.distance(x10y10) == pytest.approx(10)
    assert y10.distance(x10y10) == pytest.approx(10)
    assert x10y10.distance(origin) == pytest.approx(10 * 2 ** 0.5)
