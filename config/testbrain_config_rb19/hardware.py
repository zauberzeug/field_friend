configuration = {
    'wheels': {
        'version': 'double_wheels',
        'name': 'wheels',
        'left_back_can_address': 0x000,
        'left_front_can_address': 0x100,
        'right_back_can_address': 0x200,
        'right_front_can_address': 0x300,
        'is_left_reversed': True,
        'is_right_reversed': False,
    },
    'y_axis': {
        'version': 'axis_d1',
        'name': 'yaxis',
        'can_address': 0x60,
        'homing_acceleration': 2000,
        'homing_velocity': 40,
        'profile_acceleration': 4000,
        'profile_velocity': 400,
        'profile_deceleration': 5000,
        'min_position': 0,
        'max_position': 23000,
        'axis_offset': 23000,
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
    'flashlight': {
        'version': 'none',
    },

    'bluetooth': {
        'name': 'TestBrain',
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
}
