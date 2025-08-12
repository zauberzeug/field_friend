# Imu Calibration

The IMU is not installed in its intended orientation, that is why we have to find the correct values.
The standard coordinate frame of our robots is a right-handed one where X is forward, Y is left and Z is upward.
The built-in IMUs orientation is X-left, Y-down and Z-backward, therefore a roll rotation of -90° and a yaw rotation of 90° is needed for a Robot Brain in its default configuration, where the socket connectors show backwards.
Older robots like U4 will need an additional yaw rotation of 90° afterwards, because their Robot Brains are built in sideways.

Here is a short Python script to generate the needed configuration:

```python
#!/usr/bin/env python3
import numpy as np
from rosys.geometry import Rotation

base_rotation = Rotation.from_euler(np.deg2rad(-90), 0, np.deg2rad(90))
print(f'{base_rotation=}')

roll = np.deg2rad(0.0)
pitch = np.deg2rad(0.0)
correction = Rotation.from_euler(roll, pitch, 0.0)
print(f'{correction=}')

complete_correction = correction * base_rotation
print(f'{complete_correction=}')

print('\nCode snippet for your robot\'s configuration:')
print(f'imu=Imu(offset_rotation=Rotation.from_euler({complete_correction.roll:.6f}, {complete_correction.pitch:.6f}, {complete_correction.yaw:.6f}))')
```

Steps:

1. Find the correct base orientation of your IMU.
   If your Field Friend has the standard configuration, you can use this as a starting point:
   `Imu(offset_rotation=Rotation.from_euler(-1.570796, -0.000000, 1.570796))`
2. Roll and pitch your robot manually to check if the configuration is correct and the axes are correctly rotated.
3. Place your robot on a level surface and check the IMU's current values on the [development page](http://192.168.42.2/dev)
4. Put the shown values for roll and pitch in the script above.
   That will generate your final configuration values.
   Note, that IMU values have some noise, so your configuration won't be perfect.
