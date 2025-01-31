configuration = {
    'wheels': {
        'version': 'double_wheels',
        'name': 'wheels',
        'left_back_can_address': 0x100,
        'left_front_can_address': 0x000,
        'right_back_can_address': 0x300,
        'right_front_can_address': 0x200,
        'is_left_reversed': True,
        'is_right_reversed': False,
        'odrive_version': 6,
    },
    'y_axis': {
        'version': 'none',
    },
    'flashlight': {
        'version': 'none',
    },

    'z_axis': {
        'version': 'none',
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
        'num': 2,
    },
    'battery_control': {
        'name': 'battery_control',
        'on_expander': True,
        'reset_pin': 15,
        'status_pin': 13,
    },
    'status_control': {
        'name': 'status_control',
    },
    'bluetooth': {
        'name': 'fieldfriend-f17',
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
    'imu': {
        'name': 'imu',
        'offset_rotation': [-1.5985471, -0.0048869, 0.0],
    },
    'gnss': {
        # TODO: which antenna is main?
        'x': 0.23618,  # when back antenna is main, 0.43682 when front antenna is main
        'y': 0.0,
        'z': 1.01226,
        'yaw_deg': 0.0,
    },
}
