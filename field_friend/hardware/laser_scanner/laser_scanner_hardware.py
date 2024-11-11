import os
import subprocess
from typing import Optional

import numpy as np
import rosys
from rosys.geometry import Point

from . import LaserScanner, Scan

SEPARATOR = '---\n'


class LaserScannerHardware(LaserScanner):
    def __init__(self, serial_port: str) -> None:
        super().__init__()

        self.serial_port = serial_port
        self.process: Optional[subprocess.Popen] = None
        self.buffer: str = ''
        self.is_active = True

        rosys.on_startup(self.start)
        rosys.on_shutdown(self.tear_down)
        rosys.on_repeat(self.step, 0.02)

    async def step(self) -> None:
        if self.process is None and self.is_active:
            await rosys.run.io_bound(self.start)

        if self.process is not None:
            new_data = (self.process.stdout.read() or b'').decode()
            # self.log.warning(f'reading from process: {new_data}')
            self.buffer += new_data
            if SEPARATOR in new_data:
                data, self.buffer = self.buffer.split(SEPARATOR)[-2:]
                try:
                    data = np.array([tuple(map(float, line.split())) for line in data.splitlines()])
                    if len(data):
                        scan = self._process_data(data)
                        if len(scan.points) < 10:
                            return
                        self.latest_scan = scan
                        self.NEW_SCAN.emit(scan)
                except ValueError as e:
                    self.log.warning('Could not parse lidar data:')
                    self.log.warning(e)

        if self.process is not None and not self.is_active:
            await rosys.run.io_bound(self.stop)

    def _process_data(self, data: np.ndarray) -> Scan:
        now = rosys.time()
        points = []
        for quality, angle, distance in data:
            if quality == 0 or distance == 0:
                continue
            point = self.pose.point.polar(distance / 1000.0, np.deg2rad(angle) + self.pose.yaw)
            points.append(point)
        return Scan(time=now, points=points)

    def start(self) -> None:
        self.log.warning('turning lidar on')
        executable = '/usr/local/bin/rplidar'
        command = [executable, '--channel', '--serial', self.serial_port, '115200']
        self.process = subprocess.Popen(command, stdout=subprocess.PIPE)
        os.set_blocking(self.process.stdout.fileno(), False)

    async def tear_down(self) -> None:
        await rosys.run.io_bound(self.stop)

    def stop(self) -> None:
        self.is_active = False
        if self.process is None:
            return
        self.log.info('turning lidar off')
        try:
            self.process.terminate()
        except Exception:
            self.log.exception('could not terminate process')
        self.process = None


# --channel --serial /dev/ttyUSB0 115200
