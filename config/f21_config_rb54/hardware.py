configuration = {
    'wheels': {
        'version': 'double_uu_wheels',
        'name': 'wheels',
        'left_can_address': 12,
        'right_can_address': 11,
        'is_left_reversed': False,
        'is_right_reversed': False,
    },
    'y_axis': {
        'version': 'none'
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
    'flashlight': {
        'version': 'flashlight_pwm_v2',
        'name': 'flashlight',
        'on_expander': True,
        'front_pin': 12,
        'back_pin': 23,
    },
    'bumper': {
        'name': 'bumper',
        'on_expander': True,
        'pins': {'front_top': 21, 'front_bottom': 35, 'back': 18},
    },
    'status_control': {
        'name': 'status_control',
    },
    'bluetooth': {
        'name': 'fieldfriend-f21',
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
        'offset_rotation': [-1.6241204, 0.0017964, 0.0],
    },
    'gnss': {
        'x': 0.093,
        'y': 0.255,
        'z': 0.622,
        'yaw_deg': 0.0,
    },
}
