# this is u1
fieldfriend_configuration = {
    'params': {
        'motor_gear_ratio': 12.52,
        'thooth_count': 17,
        'pitch': 0.041,
        'wheel_distance': 0.47,
        'work_x': 0.07,
        'drill_radius': 0.025,
        'tool': 'none',
    },
    'robot_brain': {
        'flash_params': ['nand'],
    },
    'bluetooth': {
        'name': 'uckerbot-u1',
    },
    'serial': {
        'name': 'serial',
        'rx_pin': 26,
        'tx_pin': 27,
        'baud': 115200,
        'num': 1,
    },
    'expander': {
        'name': 'p0',
        'boot': 25,
        'enable': 14,
    },
    'can': {
        'name': 'can',
        'on_expander': False,
        'rx_pin': 32,
        'tx_pin': 33,
        'baud': 1_000_000,
    },
    'wheels': {
        'version': 'double_wheels',
        'name': 'wheels',
        'left_back_can_address': 0x000,
        'left_front_can_address': 0x100,
        'right_back_can_address': 0x200,
        'right_front_can_address': 0x300,
        'is_left_reversed': False,
        'is_right_reversed': True,
    },
    'y_axis': {
        'version': 'none',
    },
    'z_axis': {
        'version': 'none',
    },
    'flashlight': {
        'version': 'flashlight',
        'name': 'flashlight',
        'pin': 5,
        'on_expander': True,
    },
    'estop': {
        'name': 'estop',
        'pins': {'1': 34, '2': 35},
    },
    'bms': {
        'name': 'bms',
        'on_expander': True,
        'rx_pin': 26,
        'tx_pin': 27,
        'baud': 9600,
        'num': 1,
    },
    'imu': {
        'name': 'imu',
    },
}
