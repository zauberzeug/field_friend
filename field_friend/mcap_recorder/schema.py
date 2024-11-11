IMAGE_SCHEMA_NAME = 'sensor_msgs/Image'
IMAGE_SCHEMA_TEXT = '''
# This is a ROS2 compatible Image message definition
std_msgs/Header header
uint32 height
uint32 width
string encoding
uint8 is_bigendian
uint32 step
uint8[] data

================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id

================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
'''

LASER_SCHEMA_NAME = 'sensor_msgs/LaserScan'
LASER_SCHEMA_TEXT = '''
# Single scan from a planar laser range-finder
std_msgs/Header header
float32 angle_min        # start angle of the scan [rad]
float32 angle_max        # end angle of the scan [rad]
float32 angle_increment  # angular distance between measurements [rad]
float32 time_increment   # time between measurements [seconds]
float32 scan_time        # time between scans [seconds]
float32 range_min        # minimum range value [m]
float32 range_max        # maximum range value [m]
float32[] ranges         # range data [m]
float32[] intensities    # intensity data [device-specific units]

================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id

================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
'''

GNSS_SCHEMA_NAME = 'sensor_msgs/NavSatFix'
GNSS_SCHEMA_TEXT = '''
# Navigation Satellite fix for any Global Navigation Satellite System
std_msgs/Header header
sensor_msgs/NavSatStatus status
float64 latitude          # Latitude [degrees]. Positive is north of equator; negative is south
float64 longitude         # Longitude [degrees]. Positive is east of prime meridian; negative is west
float64 altitude          # Altitude [m]. Positive is above the WGS 84 ellipsoid
float64[9] position_covariance    # Row-major representation of the 3x3 covariance matrix
uint8 position_covariance_type    # Navigation fix type (0-1)

================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id

================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec

================================================================================
MSG: sensor_msgs/NavSatStatus
int8 status      # Status of GNSS fix
int16 service    # Satellite service being used
'''