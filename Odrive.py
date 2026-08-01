import math
from odrive.enums import *

# ============================================================
# ODrive Configuration for AS5047P Magnetic Encoder (SPI)
# ============================================================

odrv = odrv0

# DC Bus configuration
odrv.config.dc_bus_overvoltage_trip_level = 50
odrv.config.dc_bus_undervoltage_trip_level = 10.5
odrv.config.dc_max_positive_current = math.inf
odrv.config.dc_max_negative_current = -math.inf

# Brake resistor configuration
odrv.config.brake_resistor0.enable = True
odrv.config.brake_resistor0.resistance = 2
odrv.config.brake_resistor0.enable_dc_bus_voltage_feedback = True
odrv.config.brake_resistor0.dc_bus_voltage_feedback_ramp_start = 49.0
odrv.config.brake_resistor0.dc_bus_voltage_feedback_ramp_end = 49.7

# Motor configuration
odrv.axis0.config.motor.motor_type = MotorType.HIGH_CURRENT
odrv.axis0.config.motor.pole_pairs = 4
odrv.axis0.config.motor.torque_constant = 0.03180769230769231
odrv.axis0.config.motor.current_soft_max = 4.1
odrv.axis0.config.motor.current_hard_max = 8.2
odrv.axis0.config.motor.calibration_current = 4.1
odrv.axis0.config.motor.resistance_calib_max_voltage = 2
odrv.axis0.config.calibration_lockin.current = 4.1
odrv.axis0.motor.motor_thermistor.config.enabled = False

# Controller configuration
odrv.axis0.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
odrv.axis0.controller.config.input_mode = InputMode.PASSTHROUGH
odrv.axis0.controller.config.vel_limit = 140
odrv.axis0.controller.config.vel_limit_tolerance = 1.1
odrv.axis0.config.torque_soft_min = -math.inf
odrv.axis0.config.torque_soft_max = math.inf

# Watchdog configuration
odrv.axis0.config.enable_watchdog = True
odrv.axis0.config.watchdog_timeout = 0.5
odrv.axis0.config.enable_watchdog = False

# ============================================================
# AS5047P SPI Encoder Configuration (ODrive S1)
# ============================================================
# AS5047P wiring for ODrive S1:
#   MOSI -> M0_ENC_A (SPI MOSI)
#   MISO -> M0_ENC_B (SPI MISO)
#   SCK  -> M0_ENC_Z (SPI SCK)
#   CS   -> GPIO12 / nCS
#   VDD  -> 3.3V
#   GND  -> GND
#
# Note: ODrive S1 uses the AMS AS50xx SPI mode for AS5047P.
#       The chip select pin is configured below with ncs_gpio = 12.

# Enable SPI encoder on encoder0
odrv.axis0.config.encoder_bandwidth = 1000

# Configure SPI encoder (encoder0)
odrv.inc_encoder0.config.enabled = False  # Disable incremental encoder if present
odrv.axis0.config.load_encoder = EncoderId.SPI_ENCODER0
odrv.axis0.config.commutation_encoder = EncoderId.SPI_ENCODER0
odrv.spi_encoder0.config.mode = SpiEncoderMode.AMS
odrv.spi_encoder0.config.ncs_gpio = 12

# ============================================================
# UART configuration
# ============================================================
odrv.config.enable_uart_a = True
odrv.config.gpio7_mode = GpioMode.UART_A
odrv.config.gpio6_mode = GpioMode.UART_A
odrv.config.uart_a_baudrate = 115200

# ============================================================
# CAN configuration
# ============================================================
odrv.can.config.protocol = Protocol.NONE

# Save configuration and reboot
odrv.save_configuration()

# ============================================================
# Post-reboot calibration sequence (run after save_configuration)
# ============================================================
# After the ODrive reboots, run the following calibration:
#
# 1. Check encoder connection:
#    odrv.spi_encoder0.status  # Should show no errors
#    odrv.spi_encoder0.raw      # Should show changing values when rotating
#
# 2. Run motor calibration (only needed once):
#    odrv.axis0.requested_state = AxisState.MOTOR_CALIBRATION
#    # Wait for completion, check: odrv.axis0.motor.config.phase_resistance
#    #                             odrv.axis0.motor.config.phase_inductance
#
# 3. Run encoder offset calibration:
#    odrv.axis0.requested_state = AxisState.ENCODER_OFFSET_CALIBRATION
#    # Motor will rotate slowly to find encoder/motor alignment
#
# 4. Optional: Save calibration to skip motor calibration on next boot
#    odrv.axis0.motor.config.pre_calibrated = True
#    odrv.axis0.config.startup_encoder_offset_calibration = True
#    odrv.save_configuration()
#
# 5. Enter closed loop control:
#    odrv.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
#    odrv.axis0.controller.input_vel = 10  # Test with 10 turns/sec