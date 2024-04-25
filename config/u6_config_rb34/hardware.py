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
        'version': 'none'
    },
    'z_axis': {
        'version': 'none',
    },
    'flashlight': {
        'version': 'flashlight_pwm_v2',
        'name': 'flashlight',
        'on_expander': False,
        'front_pin': 5,
        'back_pin': 4,
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
    'bumper': {
        'name': 'bumper',
        'on_expander': True,
        'pins': {'front_top': 35, 'front_bottom': 18, 'back': 21},
    },
    'status_control': {
        'name': 'status_control',
    },
    'bluetooth': {
        'name': 'uckerbot-u5',
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
    }

}
