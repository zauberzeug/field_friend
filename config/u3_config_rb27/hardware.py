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
        'version': 'none',
        # 'version': 'chain_axis',
        # 'name': 'chain_axis',
        # 'min_position': -0.10,
        # 'max_position': 0.10,
        # 'motor_on_expander': False,
        # 'end_stops_on_expander': True,
        # 'step_pin': 13,
        # 'dir_pin': 4,
        # 'alarm_pin': 36,
        # 'ref_t_pin': 35,
        # 'end_stops_inverted': False,
        # 'acceleration': 1000,
        # 'quick_stop_deceleration': 4000,
    },
    'z_axis': {
        'version': 'z_axis_stepper',
        'name': 'z_axis',
        'max_speed': 60_000,
        'reference_speed': 20_000,
        'min_position': -0.197,
        'max_position': 0.00,
        'axis_offset': 0.0,
        'steps_per_m': 1600 * 1000,
        'step_pin': 33,
        'dir_pin': 4,
        'alarm_pin': 32,
        'end_t_pin': 12,
        'end_b_pin': 25,
        'motor_on_expander': True,
        'end_stops_on_expander': True,
        'reversed_direction': False,
        'end_stops_inverted': False,
        'acceleration': 1000,
        'quick_stop_deceleration': 4000,
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
    },
    'imu': {
        'name': 'imu',
        'offset_rotation': [-1.570796, 0, 0],
    },
    'gnss': {
        'x': 0.06,
        'y': -0.243,
        'z': 0.662,
        'yaw_deg': 0.0,
    },
}
