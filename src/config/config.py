from __future__ import annotations

from typing import Final

#Commands 5,6,7 are not used nows
# Movement commands to Arduino
MOVE_STOP: Final[int] = 0
MOVE_FORWARD: Final[int] = 1
MOVE_BACKWARD: Final[int] = 2
MOVE_COUNTER_CLOCKWISE: Final[int] = 3
MOVE_CLOCKWISE: Final[int] = 4
RESET_ENCODERS: Final[int] = 99

# Servo Command to Arduino
SET_SERVO_ANGLE: Final[int] = 8

# I2C Settings for communication between pi and arduiono
ARDUINO_ADDRESS: Final[int] = 0x04
I2C_BUS: Final[int] = 1

# Robot parameters
PULSES_PER_REV: Final[int] = 10  # Each rotation gives 10 encoder units, so 10 = 1 turn
CM_PER_ROTATION: Final[int] = 22  # cm traveled per wheel rotation (measured)
WHEEL_BASE: Final[int] = 16  # cm between wheels from edges
ROVER_LENGTH: Final[int] = 27  # lenght of the mars rover
