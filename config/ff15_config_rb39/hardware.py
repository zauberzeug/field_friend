configuration = {
    'wheels': {
        'version': 'double_wheels',
        'name': 'wheels',
        'left_back_can_address': 0x100,
        'left_front_can_address': 0x000,
        'right_back_can_address': 0x300,
        'right_front_can_address': 0x200,
        'is_left_reversed': False,
        'is_right_reversed': True,
        'odrive_version': 6,
    },
    # # FIXME: PIN checken
    # 'eyes': {
    #     'name': 'eyes',
    #     'on_expander': True,
    #     'eyes_pin': 12,
    # },
    'y_axis': {
        'version': 'axis_d1',
        'name': 'yaxis',
        'can_address': 0x60,
        'homing_acceleration': 2000,
        'homing_velocity': 40,
        'homing_Deceleration': 2000,
        'profile_acceleration': 4000,
        'profile_velocity': 400,
        'profile_Deceleration': 5000,
    },
    'z_axis': {
        'version': 'axis_d1',
        'name': 'zaxis',
        'can_address': 0x70,

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
        'version': 'none',
    },
    'bumper': {
        'name': 'bumper',
        'on_expander': True,
        'pins': {'front_top': 18, 'front_bottom': 35, 'back': 21},
    },
    'status_control': {
        'name': 'status_control',
    },
    'bluetooth': {
        'name': 'fieldfriend-f15',
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
