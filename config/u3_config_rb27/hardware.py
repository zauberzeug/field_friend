configuration = {'wheels': {
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
        'version': 'chain_axis',
        'name': 'chain_axis',
        'motor_on_expander': False,
        'end_stops_on_expander': True,
        'step_pin': 13,
        'dir_pin': 4,
        'alarm_pin': 36,
        'ref_t_pin': 35,
},
    'z_axis': {
        'version': 'z_axis_v2',
        'name': 'z_axis',
        'step_pin': 33,
        'dir_pin': 4,
        'alarm_pin': 32,
        'ref_t_pin': 12,
        'end_b_pin': 25,
        'motor_on_expander': True,
        'end_stops_on_expander': True,
        'ref_t_inverted': False,
        'end_b_inverted':  False,
        'ccw': False,
},
    'flashlight': {
        'version': 'flashlight_v2',
        'name': 'flashlight',
        'front_pin': 23,
        'back_pin': 22,
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
        'num': 2,
},
    'battery_control': {
        'name': 'battery_control',
        'on_expander': True,
        'reset_pin': 15,
        'status_pin': 13,
},
    'bluetooth': {
        'name': 'uckerbot-u3',
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
