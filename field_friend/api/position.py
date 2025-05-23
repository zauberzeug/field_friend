import numpy as np
from fastapi.responses import JSONResponse
from nicegui import app
from rosys.geometry import GeoPose, GeoReference

from field_friend.system import System


class Position:
    def __init__(self, system: System) -> None:
        self.system = system

        @app.get('/api/position/current')
        async def current():
            # pylint: disable=protected-access
            pose = self.system.robot_locator.pose
            lat, lon, heading = GeoPose.from_pose(pose).degree_tuple
            data = {
                'time': pose.time,
                'local': {
                    'x': pose.x,
                    'y': pose.y,
                    'yaw': pose.yaw_deg,
                },
                'uncertainty': {
                    'x': self.system.robot_locator._Sxx[0, 0],
                    'y': self.system.robot_locator._Sxx[1, 0],
                    'yaw': np.deg2rad(self.system.robot_locator._Sxx[2, 0]),
                },
                'gnss': {
                    'lat': lat,
                    'lon': lon,
                    'heading': heading,
                },
            }
            return data

        @app.get('/api/position/gnss')
        async def gnss():
            if self.system.gnss is None:
                return JSONResponse(content={'status': 'error', 'message': 'GNSS not available'}, status_code=500)
            last_measurement = self.system.gnss.last_measurement
            if last_measurement is None:
                return JSONResponse(content={'status': 'error', 'message': 'No GNSS measurement available'}, status_code=500)
            lat, lon, heading = last_measurement.pose.degree_tuple
            data = {
                'time': last_measurement.time,
                'gnss_time': last_measurement.gnss_time,
                'lat': lat,
                'lon': lon,
                'heading': heading,
                'quality': last_measurement.gps_quality.name.lower(),
                'num_satellites': last_measurement.num_satellites,
                'hdop': last_measurement.hdop,
                'altitude': last_measurement.altitude,
                'latitude_std_dev': last_measurement.latitude_std_dev,
                'longitude_std_dev': last_measurement.longitude_std_dev,
                'heading_std_dev': last_measurement.heading_std_dev,
            }
            return data

        @app.get('/api/position/gnss_reference')
        async def gnss_reference():
            if GeoReference.current is None:
                return JSONResponse(content={'status': 'error', 'message': 'GNSS Reference not available'}, status_code=500)
            lat, lon = GeoReference.current.origin.degree_tuple
            heading = np.rad2deg(GeoReference.current.direction)
            data = {
                'lat': lat,
                'lon': lon,
                'heading': heading,
            }
            return data
