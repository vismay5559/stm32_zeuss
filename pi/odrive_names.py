"""
Names for the raw ODrive values the state packet carries.

act_error and act_state arrive as bare numbers. A log line that says
"left_knee_pitch: DC_BUS_UNDER_VOLTAGE" gets acted on; one that says
"act_error[2] = 0x200" gets looked up later, usually after the next fall.

From the ODrive 0.6.12 API reference (ODriveError, AxisState) - the same tables
the STM32 leg test prints from.
"""

from __future__ import annotations

from typing import List

ODRIVE_ERRORS = (
    (0x00000001, "INITIALIZING"),
    (0x00000002, "SYSTEM_LEVEL"),
    (0x00000004, "TIMING_ERROR"),
    (0x00000008, "MISSING_ESTIMATE"),
    (0x00000010, "BAD_CONFIG"),
    (0x00000020, "DRV_FAULT"),
    (0x00000040, "MISSING_INPUT"),
    (0x00000100, "DC_BUS_OVER_VOLTAGE"),
    (0x00000200, "DC_BUS_UNDER_VOLTAGE"),
    (0x00000400, "DC_BUS_OVER_CURRENT"),
    (0x00000800, "DC_BUS_OVER_REGEN_CURRENT"),
    (0x00001000, "CURRENT_LIMIT_VIOLATION"),
    (0x00002000, "MOTOR_OVER_TEMP"),
    (0x00004000, "INVERTER_OVER_TEMP"),
    (0x00008000, "VELOCITY_LIMIT_VIOLATION"),
    (0x00010000, "POSITION_LIMIT_VIOLATION"),
    (0x00020000, "REQUESTED_CURRENT_TOO_HIGH"),
    (0x01000000, "WATCHDOG_TIMER_EXPIRED"),
    (0x02000000, "ESTOP_REQUESTED"),
    (0x04000000, "SPINOUT_DETECTED"),
    (0x08000000, "BRAKE_RESISTOR_DISARMED"),
    (0x10000000, "THERMISTOR_DISCONNECTED"),
    (0x40000000, "CALIBRATION_ERROR"),
)

AXIS_STATES = {
    0: "UNDEFINED",
    1: "IDLE",
    2: "STARTUP_SEQUENCE",
    3: "FULL_CALIBRATION_SEQUENCE",
    4: "MOTOR_CALIBRATION",
    6: "ENCODER_INDEX_SEARCH",
    7: "ENCODER_OFFSET_CALIBRATION",
    8: "CLOSED_LOOP_CONTROL",
    9: "LOCKIN_SPIN",
    10: "ENCODER_DIR_FIND",
    11: "HOMING",
    12: "ENCODER_HALL_POLARITY_CALIBRATION",
    13: "ENCODER_HALL_PHASE_CALIBRATION",
    14: "ANTICOGGING_CALIBRATION",
}


def error_names(err: int) -> List[str]:
    """Every set bit by name; bits this table does not know come back as hex."""
    err = int(err)
    names = [name for mask, name in ODRIVE_ERRORS if err & mask]
    known = 0
    for mask, _ in ODRIVE_ERRORS:
        known |= mask
    if err & ~known:
        names.append(f"0x{err & ~known:08X}")
    return names


def axis_state_name(state: int) -> str:
    return AXIS_STATES.get(int(state), f"state {int(state)}")
