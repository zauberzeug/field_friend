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
        'version': 'd1_axis',
        'name': 'yaxis',
        'can_address': 0x60,
        'max_speed': 2000,
        'reference_speed': 30,
        'min_position': -0.125,
        'max_position': 0.125,
        'axis_offset': 0.13,
        'steps_per_m': 1_666_666.667,  # 4000steps/turn motor; 1/10 gear; 0.024m/u
        'reversed_direction': False,
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
