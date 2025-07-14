import time

import odrive
import odrive.enums as enums  # noqa: PLR0402
from odrive.enums import *


def diagnose_motor_issue(odrv, axis_name):
    axis = getattr(odrv, axis_name)

    print(f"\n=== Diagnosing {axis_name} ===")
    print(f"Encoder ready: {axis.encoder.is_ready}")
    print(f"Encoder error: {axis.encoder.error}")
    print(f"Encoder mode: {axis.encoder.config.mode}")
    print(f"Encoder offset: {axis.encoder.config.phase_offset}")
    print(f"Motor calibrated: {axis.motor.is_calibrated}")
    print(f"Motor error: {axis.motor.error}")

    # Check for specific errors
    if axis.motor.error & enums.MOTOR_ERROR_UNKNOWN_PHASE_ESTIMATE:
        print('❌ MOTOR_ERROR_UNKNOWN_PHASE_ESTIMATE detected')
        print('   → Encoder needs calibration or has connection issues')

    if axis.encoder.error != 0:
        print(f"❌ Encoder error: {axis.encoder.error}")
        print('   → Check encoder connections and configuration')


def fix_encoder_calibration(odrv, axis_name):
    axis = getattr(odrv, axis_name)

    print(f"\n=== Fixing {axis_name} encoder calibration ===")

    # Step 1: Clear all errors
    print('1. Clearing errors...')
    odrv.clear_errors()

    # Step 2: Reset encoder configuration
    print('2. Resetting encoder configuration...')
    axis.encoder.config.phase_offset = 0
    axis.encoder.config.phase_offset_float = 0

    # Step 3: Run encoder offset calibration
    print('3. Starting encoder offset calibration...')
    axis.requested_state = enums.AXIS_STATE_ENCODER_OFFSET_CALIBRATION

    # Monitor encoder calibration
    while axis.current_state != enums.AXIS_STATE_IDLE:
        print(f"   Current state: {axis.current_state}")
        time.sleep(0.5)

        # Check for errors
        if axis.encoder.error != 0:
            print(f"   ❌ Encoder error: {axis.encoder.error}")
            return False

    print('   ✅ Encoder calibration complete!')

    # Step 4: Verify encoder is ready
    print('4. Verifying encoder status...')
    print(f"   Encoder ready: {axis.encoder.is_ready}")
    print(f"   Encoder offset: {axis.encoder.config.phase_offset}")

    if not axis.encoder.is_ready:
        print('   ❌ Encoder still not ready!')
        return False

    # Step 5: Recalibrate motor
    print('5. Recalibrating motor...')
    axis.requested_state = enums.AXIS_STATE_MOTOR_CALIBRATION

    # Monitor motor calibration
    while axis.current_state != enums.AXIS_STATE_IDLE:
        print(f"   Current state: {axis.current_state}")
        time.sleep(0.5)

        # Check for motor errors
        if axis.motor.error != 0:
            print(f"   ❌ Motor error: {axis.motor.error}")
            return False

    print('   ✅ Motor calibration complete!')

    return True


def run_full_calibration_sequence(odrv, axis_name):
    axis = getattr(odrv, axis_name)

    print(f"\n=== Running full calibration sequence for {axis_name} ===")

    # Clear everything and start fresh
    print('1. Clearing errors and resetting configuration...')
    odrv.clear_errors()
    axis.encoder.config.phase_offset = 0
    axis.encoder.config.phase_offset_float = 0

    # Run full calibration sequence
    print('2. Starting full calibration sequence...')
    axis.requested_state = enums.AXIS_STATE_FULL_CALIBRATION_SEQUENCE

    # Monitor the entire process
    while axis.current_state != enums.AXIS_STATE_IDLE:
        print(f"   Current state: {axis.current_state}")
        time.sleep(0.5)

        # Check for errors
        if axis.encoder.error != 0:
            print(f"   ❌ Encoder error: {axis.encoder.error}")
            return False
        if axis.motor.error != 0:
            print(f"   ❌ Motor error: {axis.motor.error}")
            return False

    print('   ✅ Full calibration sequence complete!')
    return True


# Main execution
if __name__ == '__main__':
    # Assuming you have odrv already connected
    # odrv = find_any()  # or however you connect to your ODrive

    odrv = odrive.find_any()

    print('=== Initial Diagnosis ===')
    # diagnose_motor_issue(odrv, "axis0")  # Working motor
    diagnose_motor_issue(odrv, 'axis1')  # Non-working motor

    # Try the step-by-step fix first
    print('\n=== Attempting Step-by-Step Fix ===')
    success = fix_encoder_calibration(odrv, 'axis1')

    if not success:
        print('\n=== Step-by-step failed, trying full calibration sequence ===')
        success = run_full_calibration_sequence(odrv, 'axis1')

    if success:
        print('\n=== Final Diagnosis ===')
        # diagnose_motor_issue(odrv, "axis0")
        diagnose_motor_issue(odrv, 'axis1')

        print('\n=== Testing Motor Operation ===')
        # Test if the motor can now be controlled
        try:
            odrv.axis1.requested_state = enums.AXIS_STATE_CLOSED_LOOP_CONTROL
            time.sleep(1)
            print(f"Axis1 state: {odrv.axis1.current_state}")

            if odrv.axis1.current_state == enums.AXIS_STATE_CLOSED_LOOP_CONTROL:
                print('✅ Motor is now working!')
                # Return to idle
                odrv.axis1.requested_state = enums.AXIS_STATE_IDLE
            else:
                print('❌ Motor still not working properly')

        except Exception as e:
            print(f"❌ Error testing motor: {e}")
    else:
        print('\n❌ Calibration failed. Check hardware connections and try again.')

    print('\n=== Final Error Check ===')
    from odrive.utils import dump_errors
    dump_errors(odrv)
