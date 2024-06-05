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
        'version': 'y_axis_canopen',
        'name': 'yaxis',
        'can_address': 0x60,
        'max_speed': 500,
        'reference_speed': 40,
        'min_position': -0.0625,
        'max_position': 0.0625,
        'axis_offset': 0.0675,
        'steps_per_m': 1_481_481.48,  # 4000steps/turn motor; 1/20 gear; 0.054m/u
        'end_r_pin': 4,
        'end_l_pin': 13,
        'motor_on_expander': False,
        'end_stops_on_expander': False,
        'reversed_direction': False,
        'end_stops_inverted': True,
        'acceleration': 1000,
        'quick_stop_deceleration': 4000,
    },
    'z_axis': {
        'version': 'tornado v1.1',
        'name': 'tornado',
        'min_position': -0.085,
        'z_can_address': 0x500,
        'turn_can_address': 0x400,
        'm_per_tick': 0.025/12.52,
        'end_top_pin': 33,  # p0
        'end_top_pin_expander': True,
        'end_bottom_pin': 4,  # p0
        'end_bottom_pin_expander': True,  # p0
        'ref_motor_pin': 5,
        'ref_motor_pin_expander': False,
        'ref_gear_pin': 36,
        'ref_gear_pin_expander': False,
        'ref_knife_stop_pin': 5,  # po
        'ref_knife_stop_pin_expander': True,  # po
        'ref_knife_ground_pin': 32,  # p0
        'ref_knife_ground_pin_expander': True,  # p0
        'motors_on_expander': False,
        'end_stops_on_expander': False,
        'is_z_reversed': True,
        'is_turn_reversed': True,
        'speed_limit': 1.5,
        'turn_speed_limit': 1.5,
        'current_limit': 30,
        'z_reference_speed': 0.0075,
        'turn_reference_speed': 0.25,
    },
    'flashlight': {
        'version': 'flashlight_pwm_v2',
        'name': 'flashlight',
        'on_expander': True,
        'front_pin': 22,
        'back_pin': 23,
    },
    # 'eyes': {
    #     'name': 'eyes',
    #     'on_expander': True,
    #     'eyes_pin': 12,
    # },
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
        'name': 'fieldfriend-f13',
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
    'small_safety': {},
}
