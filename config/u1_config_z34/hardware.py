configuration = {
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
