configuration = {
    'wheels': {
        'version': 'wheels',
        'name': 'wheels',
        'left_can_address': 0x000,
        'right_can_address': 0x100,
        'm_per_tick':0.1,
        'width':0.21,
        'is_left_reversed': True,
        'is_right_reversed': False,
    },
    'y_axis': {
        'version': 'none',

    },
    'z_axis': {
        'version': 'none',

    },
    'flashlight': {
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
        'on_expander': False,
        'reset_pin': 15,
        'status_pin': 13,
    },
    'status_control': {
        'name': 'status_control',
    },
    'bluetooth': {
        'name': 'minibot-z33',
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
