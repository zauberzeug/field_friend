#!/usr/bin/env python3
import inspect
import sys
import time

import odrive
import odrive.enums as enums  # noqa: PLR0402

odrv0 = odrive.find_any()


def assert_equal(a, b):
    try:
        assert a == b
    except AssertionError:
        frame = inspect.currentframe().f_back
        line = inspect.stack()[1].code_context[0].strip()
        arguments = tuple(line.replace('assert_equal(', '').replace(')', '').split(', '))
        print('  > %s == %s != %s' % (arguments[0], eval(arguments[0], frame.f_globals, frame.f_locals), arguments[1]))
        exec('%s = %s' % arguments, frame.f_globals, frame.f_locals)

    except Exception as e:
        print(e)


print('ODrive...')
assert_equal(odrv0.config.dc_bus_overvoltage_trip_level, 30)
assert_equal(odrv0.config.max_regen_current, 20)
assert_equal(odrv0.config.dc_max_positive_current, 50)
assert_equal(odrv0.config.dc_max_negative_current, -20)
assert_equal(odrv0.config.brake_resistance, 0)
assert_equal(odrv0.can.config.baud_rate, 1_000_000)
assert_equal(odrv0.axis0.config.can.node_id, 0x400 >> 5)
assert_equal(odrv0.axis1.config.can.node_id, 0x500 >> 5)
assert_equal(odrv0.axis0.config.can.heartbeat_rate_ms, 1000)
assert_equal(odrv0.axis1.config.can.heartbeat_rate_ms, 1000)
assert_equal(odrv0.axis0.config.can.encoder_rate_ms, 10)
assert_equal(odrv0.axis1.config.can.encoder_rate_ms, 10)

for i, axis in enumerate([odrv0.axis0, odrv0.axis1]):
    print(f'Axis {i}...')

    print('- Motor configuration')
    assert_equal(axis.motor.config.pole_pairs, 8)
    assert_equal(axis.motor.config.calibration_current, 15)
    assert_equal(axis.motor.config.current_lim, 35)
    assert_equal(axis.motor.config.current_lim_margin, 15)
    assert_equal(axis.motor.config.torque_constant, 1.4500000476837158)
    assert_equal(axis.motor.config.torque_lim, 100)
    assert_equal(axis.motor.config.motor_type, enums.MOTOR_TYPE_HIGH_CURRENT)
    assert_equal(axis.motor.config.current_control_bandwidth, 2)
    assert_equal(axis.motor.config.resistance_calib_max_voltage, 10)
    assert_equal(axis.motor.fet_thermistor.config.temp_limit_lower, 60)
    assert_equal(axis.motor.fet_thermistor.config.temp_limit_upper, 70)

    print('- Encoder configuration')
    assert_equal(axis.encoder.config.mode, enums.ENCODER_MODE_HALL)
    assert_equal(axis.encoder.config.cpr, 48)
    assert_equal(axis.encoder.config.bandwidth, 40)

    print('- Controller configuration')
    assert_equal(axis.controller.config.enable_vel_limit, True)
    assert_equal(axis.controller.config.enable_torque_mode_vel_limit, False)
    assert_equal(axis.config.startup_motor_calibration, False)
    assert_equal(axis.config.startup_encoder_offset_calibration, False)
    assert_equal(axis.config.startup_closed_loop_control, True)
    assert_equal(axis.controller.config.input_mode, enums.INPUT_MODE_PASSTHROUGH)
    assert_equal(axis.controller.config.vel_gain, 20.0)
    assert_equal(axis.controller.config.vel_integrator_gain, 40.0)
    assert_equal(axis.controller.config.vel_differentiator_gain, 0.8999999761581421)
    assert_equal(axis.controller.config.vel_limit, 20)
    assert_equal(axis.controller.config.pos_gain, 2.200000047683716)
    assert_equal(axis.controller.config.control_mode, enums.CONTROL_MODE_VELOCITY_CONTROL)
    assert_equal(axis.controller.config.enable_overspeed_error, False)
    assert_equal(axis.controller.config.spinout_mechanical_power_threshold, -1000)
    assert_equal(axis.controller.config.spinout_electrical_power_threshold, 1000)

try:
    odrv0.save_configuration()
except:
    pass
finally:
    time.sleep(1.0)
    odrv0 = odrive.find_any()

for i, axis in enumerate([odrv0.axis0, odrv0.axis1]):
    print('- Calibration...')
    axis.requested_state = enums.AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    time.sleep(0.1)
    while axis.current_state != enums.AXIS_STATE_IDLE:
        time.sleep(0.1)
    print('  Done.')

    assert_equal(axis.motor.config.pre_calibrated, True)
    assert_equal(axis.encoder.config.pre_calibrated, True)


try:
    odrv0.save_configuration()
except:
    sys.exit(0)
