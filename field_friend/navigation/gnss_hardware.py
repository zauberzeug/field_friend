import datetime
from typing import Optional

import pynmea2
import rosys
import serial
from rosys.driving.odometer import Odometer
from serial.tools import list_ports

from .gnss import Gnss, GNSSRecord


class GnssHardware(Gnss):
    PORT = '/dev/cu.usbmodem36307295'
    TYPES_NEEDED = {'GGA', 'GNS', 'HDT'}

    def __init__(self, odometer: Odometer, antenna_offset: float) -> None:
        super().__init__(odometer, antenna_offset)
        self.ser: serial.Serial | None = None

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

    async def _create_new_record(self) -> Optional[GNSSRecord]:
        if self.ser is None:
            return None
        record = GNSSRecord()
        types_seen: set[str] = set()
        try:
            while self.TYPES_NEEDED != types_seen:
                line = await self._read()
                if not line:
                    self.log.debug('No data received')
                    return None
                try:
                    msg = pynmea2.parse(line)
                    if not hasattr(msg, 'sentence_type'):
                        self.log.debug(f'No sentence type: {msg}')
                        return None
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
                    if msg.sentence_type == 'HDT' and getattr(msg, 'heading', None):
                        record.heading = float(msg.heading) if msg.heading else None
                except pynmea2.ParseError as e:
                    self.log.info(f'Parse error: {e}')
                    continue
        except serial.SerialException as e:
            self.log.info(f'Device error: {e}')
            self.device = None
            return None
        return record
