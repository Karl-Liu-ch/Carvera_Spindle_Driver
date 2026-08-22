"""ODrive S1 0.6.11 configuration for the Carvera Nano R4 interface.

Run this file from odrivetool with the motor mechanically unloaded.  The
script writes persistent configuration and calls save_configuration(), which
reboots the ODrive.  It intentionally does not start calibration or spin the
motor automatically; follow the commissioning commands at the bottom after
the reboot.
"""

import math

from odrive.enums import *


odrv = odrv0
axis = odrv.axis0

# Keep the power stage disabled while changing feedback and interface settings.
axis.requested_state = AxisState.IDLE
axis.controller.input_vel = 0.0

# ---------------------------------------------------------------------------
# DC bus and brake resistor
# ---------------------------------------------------------------------------

odrv.config.dc_bus_overvoltage_trip_level = 50.0
odrv.config.dc_bus_undervoltage_trip_level = 10.5
odrv.config.dc_max_positive_current = math.inf
odrv.config.dc_max_negative_current = -math.inf

odrv.config.brake_resistor0.enable = True
odrv.config.brake_resistor0.resistance = 2.0
odrv.config.brake_resistor0.enable_dc_bus_voltage_feedback = True
odrv.config.brake_resistor0.dc_bus_voltage_feedback_ramp_start = 49.0
odrv.config.brake_resistor0.dc_bus_voltage_feedback_ramp_end = 49.7

# ---------------------------------------------------------------------------
# Motor
# ---------------------------------------------------------------------------

axis.config.motor.motor_type = MotorType.HIGH_CURRENT
axis.config.motor.pole_pairs = 4
axis.config.motor.torque_constant = 0.03180769230769231
axis.config.motor.current_soft_max = 4.1
axis.config.motor.current_hard_max = 8.2
axis.config.motor.calibration_current = 4.1
axis.config.motor.resistance_calib_max_voltage = 2.0
axis.config.calibration_lockin.current = 4.1
axis.motor.motor_thermistor.config.enabled = False

# ---------------------------------------------------------------------------
# Hall feedback
# ---------------------------------------------------------------------------
#
# ODrive S1 wiring:
#   Hall supply -> 5V
#   Hall A      -> HALL_A
#   Hall B      -> HALL_B
#   Hall C      -> HALL_C
#   Hall GND    -> GND
#
# Hall feedback has only six states per electrical revolution.  ODrive
# recommends a lower encoder bandwidth than is normally used with an SPI or
# incremental encoder.

odrv.inc_encoder0.config.enabled = False
odrv.hall_encoder0.config.enabled = True
axis.config.load_encoder = EncoderId.HALL_ENCODER0
axis.config.commutation_encoder = EncoderId.HALL_ENCODER0
axis.config.encoder_bandwidth = 100.0

# The Nano R4 owns the state transitions.  The ODrive must therefore boot into
# IDLE instead of calibrating or entering closed loop by itself.
axis.config.startup_motor_calibration = False
axis.config.startup_encoder_index_search = False
axis.config.startup_encoder_offset_calibration = False
axis.config.startup_closed_loop_control = False

# ---------------------------------------------------------------------------
# Velocity controller
# ---------------------------------------------------------------------------
#
# These values match ODRIVE_NANO_R4_CAN_HALL_v4:
#   ControlMode.VELOCITY_CONTROL = 2
#   InputMode.VEL_RAMP           = 2
#   MAX_MOTOR_RPM                = 8400 rpm = 140 turns/s
#
# The Arduino also sends Set_Controller_Mode during startup.  Persisting the
# same values here makes the ODrive safe and deterministic before that frame is
# received.  Tune vel_ramp_rate for the actual spindle inertia and braking
# system; 100 turns/s^2 reaches 8400 rpm in 1.4 seconds.

axis.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
axis.controller.config.input_mode = InputMode.VEL_RAMP
axis.controller.config.vel_ramp_rate = 100.0
axis.controller.config.vel_limit = 140.0
axis.controller.config.vel_limit_tolerance = 1.1
axis.config.torque_soft_min = -math.inf
axis.config.torque_soft_max = math.inf

# Every Set_Input_Vel frame from the Nano R4 feeds this watchdog.  The sketch
# normally sends a velocity command every 50 ms, so 0.5 s leaves ample margin
# while still disarming the ODrive if the controller freezes or CAN is lost.
axis.config.enable_watchdog = True
axis.config.watchdog_timeout = 0.5

# ---------------------------------------------------------------------------
# Disable UART A and release its S1 GPIO pins
# ---------------------------------------------------------------------------

odrv.config.enable_uart_a = False
odrv.config.gpio6_mode = GpioMode.DIGITAL
odrv.config.gpio7_mode = GpioMode.DIGITAL

# ---------------------------------------------------------------------------
# CANSimple -- must match ODRIVE_NANO_R4_CAN_HALL_v4
# ---------------------------------------------------------------------------
#
# The Nano R4 Arduino_CAN API cannot transmit RTR frames, so all telemetry used
# by the sketch must be sent cyclically by the ODrive.  Keep Classic CAN (no
# CAN-FD bit-rate switching), standard 11-bit IDs, node 0 and 1 Mbit/s.

axis.config.can.node_id = 0
axis.config.can.heartbeat_msg_rate_ms = 100
axis.config.can.encoder_msg_rate_ms = 10
axis.config.can.iq_msg_rate_ms = 10
axis.config.can.bus_voltage_msg_rate_ms = 100
axis.config.can.error_msg_rate_ms = 100

odrv.can.config.baud_rate = 1_000_000
odrv.can.config.data_baud_rate = 0
odrv.can.config.tx_brs = False
odrv.can.config.protocol = Protocol.SIMPLE

# Persist all settings.  save_configuration() reboots the ODrive and normally
# disconnects the current odrivetool object.
odrv.save_configuration()


# ---------------------------------------------------------------------------
# Commissioning after the reboot -- run manually in odrivetool
# ---------------------------------------------------------------------------
#
# WARNING: The motor rotates during calibration.  Remove the tool and belt/load
# or otherwise make sure the rotor can turn freely.  Keep an emergency power
# disconnect within reach.
#
# For a new motor/Hall installation, run the complete sequence:
#
#   odrv0.axis0.requested_state = AxisState.FULL_CALIBRATION_SEQUENCE
#
# Wait until axis0.current_state returns to IDLE, then check:
#
#   dump_errors(odrv0)
#   odrv0.axis0.procedure_result
#   odrv0.hall_encoder0.config.hall_polarity_calibrated
#   odrv0.hall_encoder0.config.edges_calibrated
#
# FULL_CALIBRATION_SEQUENCE on firmware 0.6.11 performs motor calibration,
# Hall polarity calibration and Hall phase calibration.  It does not use an
# encoder index search.  If every result is valid, save the calibration:
#
#   odrv0.save_configuration()
#
# After the reboot, reconnect odrivetool and perform a low-speed direction test
# before connecting the spindle mechanically.  The Nano R4 firmware sends
# positive velocity when DIRECTION_SIGN is +1.0:
#
#   odrv0.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
#   odrv0.axis0.controller.input_vel = 5.0
#   odrv0.axis0.controller.input_vel = 0.0
#   odrv0.axis0.requested_state = AxisState.IDLE
