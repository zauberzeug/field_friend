import datetime
from copy import deepcopy
from typing import Optional

import numpy as np
import pynmea2
import rosys
import serial
from serial.tools import list_ports

from field_friend.navigation.gnss import Gnss, GNSSRecord
from field_friend.navigation.point_transformation import get_new_position, wgs84_to_cartesian


class GnssHardware(Gnss):
    PORT = '/dev/cu.usbmodem36307295'
    TYPES_NEEDED = {'GGA', 'GNS', 'HDT'}

    def __init__(self, odometer: rosys.driving.Odometer, antenna_offset: float) -> None:
        super().__init__(odometer)
        self.antenna_offset = antenna_offset

    def __del__(self) -> None:
        if self.ser is not None:
            self.ser.close()

    async def try_connection(self) -> None:
        if self.device is not None:
            return
        # self.log.info('Searching for GNSS device...')
        for port in list_ports.comports():
            self.log.info(f'Found port: {port.device} - {port.description}')
            if 'Septentrio' in port.description:
                self.log.info(f'Found GNSS device: {port.device}')
                self.device = port.device
                break
        else:
            self.device = None
            # self.log.error('No GNSS device found')
            return

        self.log.info(f'Connecting to GNSS device "{self.device}"...')
        try:
            self.ser = serial.Serial(self.device, baudrate=115200, timeout=0.2)
        except serial.SerialException as e:
            self.log.error(f'Could not connect to GNSS device: {e}')
            self.device = None
        self.log.info(f'Connected to GNSS device "{self.device}"')

    async def _read(self) -> Optional[str]:
        if self.ser is None:
            self.log.debug('GNSS device not connected')
            return None
        if not self.ser.isOpen():
            self.log.debug('GNSS device not open')
            return None
        line = await rosys.run.io_bound(self.ser.read_until, b'\r\n')
        if not line:
            self.log.debug('No data')
            return None
        line = line.decode()
        return line

    async def update(self) -> None:
        await super().update()
        if self.ser is None:
            return
        record = GNSSRecord()
        has_location = False
        has_heading = False

        types_seen: set[str] = set()
        try:
            while self.TYPES_NEEDED != types_seen:
                line = await self._read()
                if not line:
                    self.log.debug('No data received')
                    return
                try:
                    msg = pynmea2.parse(line)
                    if not hasattr(msg, 'sentence_type'):
                        self.log.debug(f'No sentence type: {msg}')
                        return
                    if msg.sentence_type in self.TYPES_NEEDED:
                        types_seen.add(msg.sentence_type)
                    if msg.sentence_type == 'GGA' and getattr(msg, 'gps_qual', 0) > 0:
                        # self.log.info(f'GGA: gps_qual: {msg.gps_qual}, lat:{msg.latitude} and long:{msg.longitude}')
                        record.gps_qual = msg.gps_qual
                        record.altitude = msg.altitude
                        record.separation = msg.geo_sep
                    if msg.sentence_type == 'GNS' and getattr(msg, 'mode_indicator', None):
                        # self.log.info(f'GNS: mode: {msg.mode_indicator}, lat:{msg.latitude} and long:{msg.longitude}')
                        if isinstance(msg.timestamp, datetime.time):
                            dt = datetime.datetime.combine(datetime.date.today(), msg.timestamp)
                            dt.replace(tzinfo=datetime.timezone.utc)
                            record.timestamp = dt.timestamp()
                        else:
                            record.timestamp = msg.timestamp
                        record.latitude = msg.latitude
                        record.longitude = msg.longitude
                        record.mode = msg.mode_indicator
                        # print(f'The GNSS message: {msg.mode_indicator}')
                        has_location = True
                    if msg.sentence_type == 'HDT' and getattr(msg, 'heading', None):
                        record.heading = float(msg.heading)
                        has_heading = True
                except pynmea2.ParseError as e:
                    self.log.info(f'Parse error: {e}')
                    continue
        except serial.SerialException as e:
            self.log.info(f'Device error: {e}')
            self.device = None
            return
        if self.record.gps_qual > 0 and record.gps_qual == 0:
            self.log.warning('GNSS lost')
            self.GNSS_CONNECTION_LOST.emit()
        if self.record.gps_qual == 4 and record.gps_qual != 4:
            self.log.warning('GNSS RTK fix lost')
            self.RTK_FIX_LOST.emit()
        self.record = deepcopy(record)
        if has_location:
            if record.gps_qual == 4:  # 4 = RTK fixed, 5 = RTK float
                if self.reference_lat is None or self.reference_lon is None:
                    self.log.info(f'GNSS reference set to {record.latitude}, {record.longitude}')
                    self.set_reference(record.latitude, record.longitude)
                else:
                    if has_heading:
                        yaw = np.deg2rad(-record.heading)
                    else:
                        yaw = self.odometer.get_pose(time=record.timestamp).yaw
                        # TODO: Better INS implementation if no heading provided by GNSS
                    # correct the gnss coordinat by antenna offset
                    corrected_coordinates = get_new_position([
                        record.latitude, record.longitude], self.antenna_offset, yaw+np.pi/2)
                    self.record.latitude = deepcopy(corrected_coordinates[0])
                    self.record.longitude = deepcopy(corrected_coordinates[1])
                    cartesian_coordinates = wgs84_to_cartesian([self.reference_lat, self.reference_lon], [
                        self.record.latitude, self.record.longitude])
                    pose = rosys.geometry.Pose(
                        x=cartesian_coordinates[0],
                        y=cartesian_coordinates[1],
                        yaw=yaw,
                        time=record.timestamp,
                    )
                    distance = self.odometer.prediction.distance(pose)
                    if distance > 1:
                        self.log.warning(f'GNSS distance to prediction to high: {distance:.2f}m!!')
                    self.ROBOT_POSE_LOCATED.emit(pose)
                    self.ROBOT_POSITION_LOCATED.emit()
            elif record.gps_qual == 0:
                return
            else:
                self.ROBOT_POSITION_LOCATED.emit()

    def set_reference(self, lat: float, lon: float) -> None:
        self.reference_lat = lat
        self.reference_lon = lon
