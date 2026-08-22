# Carvera Spindle Driver

Converts the Carvera CNC spindle PWM command into velocity commands for an ODrive S1 over CANSimple or a SOLO PICO over CANopen, then returns measured motor speed and fault status to the Carvera controller.

> [!WARNING]
> This experimental hardware and firmware controls a high-speed spindle. Remove the cutting tool during initial commissioning, limit bus voltage and motor current, and verify that the emergency stop can remove drive power. The current limits, speed limits, direction, gear ratio, and protection thresholds in this repository are specific to the present prototype. Do not use them with another motor or mechanical system without verification. This project is not a substitute for an independent hardware emergency stop, overspeed protection, or a physical machine enclosure.

## Overview

```text
Carvera                       Arduino Nano R4               Motor drive
--------                      ---------------               -----------
Spindle PWM (~1 kHz) -------> D2  Duty capture  ----------> CAN/CANopen velocity
Blue speed feedback  <------- D3  12 pulses/rev <---------- Measured motor speed
Red alarm input      <------- D6  Open-drain alarm <------- Communication/drive/speed/load faults
                                      |
                                      +---- USB serial status and error clearing
```

Main features:

- Maps PWM duty linearly to a spindle speed target, then converts spindle speed to motor speed using the configured gear ratio.
- Supports ODrive S1 + Hall + CANSimple and SOLO PICO + Hall + CANopen drive configurations.
- Generates the Carvera Blue speed-feedback signal from measured motor speed at 12 pulses per revolution.
- Implements the Red alarm as active-low on fault and high-impedance during normal operation.
- Monitors communication, drive errors, start failure, speed drop, and the `|Iq| × motor RPM` load index.
- Stops Blue feedback, stops the drive, and signals the Carvera when a serious fault occurs.
- Prints periodic diagnostics over 115200-baud USB serial and accepts status and error-clear commands.
- Includes an ESP32-C6 1 kHz PWM generator for bench testing without a Carvera controller.

## Repository Layout

| Path | Purpose | Current key configuration |
| --- | --- | --- |
| `ODRIVE_NANO_R4_CAN_HALL_v4/` | Nano R4 to ODrive S1 CANSimple interface | Hall, node 0, 1 Mbit/s, 12,500 spindle RPM |
| `SOLO_NANO_R4_CAN_v2/` | Nano R4 to SOLO PICO CANopen interface | Hall, 1 Mbit/s, 18,000 spindle RPM |
| `ESP32_C6_PWM_TEST/` | USB-controlled 1 kHz PWM test source | GPIO4, 8-bit, 0–15,000 RPM |
| `Odrive.py` | ODrive S1 firmware 0.6.11 configuration script | Hall, CANSimple, node 0, 1 Mbit/s, UART disabled |

> [!IMPORTANT]
> `Odrive.py` writes power, brake-resistor, motor, Hall, velocity-controller, watchdog, and CAN settings. It then calls `save_configuration()`, which reboots the ODrive. Verify every motor, supply, and brake-resistor value before running it. The script deliberately does not start any calibration procedure that would rotate the motor.

## Requirements

### Hardware

- A Carvera CNC or an equivalent approximately 1 kHz PWM command source
- Arduino Nano R4
- An external CAN transceiver compatible with the Nano R4 native CAN controller
- One 120 Ω termination resistor at each physical end of the CAN bus
- ODrive S1 or SOLO PICO, motor, Hall sensors, and a suitable power/braking system
- USB connection for compilation, upload, and serial monitoring

### Arduino Dependencies

The Nano R4 firmware uses:

- An Arduino board core that supports the Nano R4
- `Arduino_CAN`
- `pwm.h` from the Nano R4/Renesas core
- `SOLOMotorControllers` **5.5.0** for the SOLO build

`SOLO_NANO_R4_CAN_v2_compat.cpp` is a compatibility layer for SOLOMotorControllers 5.5.0. That library version omits `ARDUINO_NANO_R4` from its native-CAN compilation guard, so the project includes the RA4M1 implementation locally in the sketch directory. If a newer SOLO library produces duplicate definitions, first check whether upstream now supports the Nano R4 directly, then remove or disable the local compatibility file.

The ESP32 test sketch requires an ESP32 Arduino core with the newer LEDC API, `ledcAttach(pin, frequency, resolution)`. The following Arduino IDE option must be enabled:

```text
Tools > USB CDC On Boot > Enabled
```

## Wiring

### Nano R4, Carvera, and CAN Transceiver

The ODrive and SOLO sketches use the same Nano R4 pins:

| Nano R4 | Direction | Connection | Description |
| --- | --- | --- | --- |
| D2 | Input | Carvera spindle PWM | Approximately 1 kHz expected; accepted range is about 667–1429 Hz (700–1500 µs period) |
| D3 | Output | Carvera Blue | Measured motor-speed feedback, 12 pulses/rev, 50% duty |
| D4 | Output | CAN transceiver TXD | Nano R4 native CAN TX |
| D5 | Input | CAN transceiver RXD | Nano R4 native CAN RX |
| D6 | Output/high-Z | Carvera Red | Pulled LOW on fault; switched to high-impedance input when normal; never actively driven HIGH |
| GND | — | All device grounds | Carvera, Nano R4, transceiver, and motor drive must share ground |

Connect the transceiver CANH/CANL pins to the motor drive CANH/CANL pins. Install one 120 Ω termination resistor at each physical end of the bus. D4 and D5 are logic-side signals and must never be connected directly to CANH or CANL.

Before connecting the Carvera, measure its PWM, Blue, and Red voltage levels and verify its internal pull-up arrangement. Confirm compatibility with the Nano R4 or the external interface circuitry. If the voltage domains differ, use a level shifter, optocoupler, or open-drain transistor. Do not rely on an MCU pin to tolerate an out-of-spec voltage.

### ESP32-C6 PWM Test Source

| ESP32-C6 | Connection | Description |
| --- | --- | --- |
| GPIO4 | Nano R4 D2 | 1 kHz, 3.3 V PWM |
| GND | Nano R4 GND | Common ground required |

ESP32-C6 GPIO is not 5 V tolerant. At power-up, the test sketch defaults to 7,500 RPM, corresponding to 50% duty. Do not assume GPIO4 is LOW while wiring or servicing the system.

## Control Mapping

Both main sketches apply a first-order filter to PWM duty and then use:

```text
requested_spindle_rpm = filtered_duty × MAX_SPINDLE_RPM
requested_motor_rpm   = requested_spindle_rpm ÷ GEAR_RATIO
blue_feedback_hz      = |measured_motor_rpm| ÷ 60 × 12
```

Current parameters:

| Parameter | ODrive | SOLO |
| --- | ---: | ---: |
| `GEAR_RATIO` | 1.635 | 1.635 |
| `MAX_SPINDLE_RPM` | 12,500 RPM | 18,000 RPM |
| `MAX_MOTOR_RPM` | 8,400 RPM | 11,500 RPM |
| Start duty threshold | 0.2% | 0.2% |
| Zero-speed hold before drive release | 4 s | 11 s |
| Power-index limit | 58,000 A·RPM | 52,000 A·RPM |

Blue feedback is generated directly from measured **motor** speed. The source comments state that the Carvera applies the transmission-ratio conversion itself. If the pulley ratio, maximum speed, or feedback pulse count changes, also review the spindle settings on the Carvera.

## ODrive S1 Configuration

### Preparing the ODrive

The firmware targets ODrive S1 firmware **0.6.11**. Before uploading the Arduino sketch, use `odrivetool` to configure and commission the motor and Hall sensors.

The included `Odrive.py` configures Hall feedback, velocity control with velocity ramp, node ID `0`, Classic CAN at 1 Mbit/s, all required cyclic telemetry, and a 0.5-second watchdog. It also disables UART A and restores GPIO6/GPIO7 to digital mode. Run it from `odrivetool` with:

```python
exec(open("Odrive.py").read())
```

The ODrive reboots after the script saves its configuration. Reconnect `odrivetool`, mechanically unload the motor, make sure it can rotate safely, and manually run:

```python
odrv0.axis0.requested_state = AxisState.FULL_CALIBRATION_SEQUENCE
# Wait until current_state returns to IDLE.
dump_errors(odrv0)
odrv0.axis0.procedure_result
odrv0.hall_encoder0.config.hall_polarity_calibrated
odrv0.hall_encoder0.config.edges_calibrated
odrv0.save_configuration()
```

On firmware 0.6.11, the full sequence performs motor calibration, Hall polarity calibration, and Hall phase calibration. Hall feedback does not use an encoder index search. Perform a low-speed direction test only after all results have been verified.

The essential Hall selection is:

```python
odrv0.hall_encoder0.config.enabled = True
odrv0.axis0.config.load_encoder = EncoderId.HALL_ENCODER0
odrv0.axis0.config.commutation_encoder = EncoderId.HALL_ENCODER0
```

Commissioning checklist:

1. Connect Hall A/B/C to the S1 `HALL_A`, `HALL_B`, and `HALL_C` inputs.
2. Complete Hall polarity and phase calibration. Do not configure an encoder index search for this setup.
3. Configure and save velocity control, CANSimple, node ID `0`, and 1 Mbit/s CAN.
4. Verify motor parameters, controller gains, current limits, and the velocity-ramp rate. The Nano R4 does not configure all of these values.
5. Enable cyclic telemetry because the current Arduino CAN API cannot transmit RTR request frames.

Required cyclic message rates:

```python
odrv0.axis0.config.can.heartbeat_msg_rate_ms = 100       # Or faster
odrv0.axis0.config.can.encoder_msg_rate_ms = 10
odrv0.axis0.config.can.iq_msg_rate_ms = 10
odrv0.axis0.config.can.bus_voltage_msg_rate_ms = 100     # Or faster
odrv0.axis0.config.can.error_msg_rate_ms = 100           # Or faster
odrv0.save_configuration()
```

ODrive property paths can change between firmware releases. Verify each field in the `odrivetool` version paired with the installed firmware. Cyclic `Get_Error` telemetry is required: the Nano R4 uses it to preserve the disarm reason and to verify that faults have actually cleared before releasing its local fault latch.

### Startup Behavior

At startup, the Nano R4 sends zero velocity, requests IDLE, and selects velocity control with velocity ramp. Initialization succeeds only when Heartbeat, Encoder, and Iq telemetry are fresh, the axis is in IDLE, and no ODrive error is active. The Nano requests CLOSED_LOOP_CONTROL and sends a nonzero velocity only after a nonzero PWM command arrives.

If motor direction is reversed, first remove the tool and perform a low-speed test, then change:

```cpp
static constexpr float DIRECTION_SIGN = -1.0f;
```

Do not attempt to reverse direction merely by swapping Hall wires. This can invalidate the calibrated phase relationship.

## SOLO PICO Configuration

### Preparing the SOLO

Commission the motor and Hall sensors in SOLO Motion Terminal/Monitor, save the configuration, and select 1 Mbit/s CANopen. At every startup, the sketch places the drive in DISABLE, writes zero speed, and restores or verifies these runtime settings:

- Digital command mode, BLDC/PMSM motor, Hall feedback, and speed mode
- Current limit: 8.2 A
- Current PI: Kp 0.3105163, Ki 0.02108
- Speed PI: Kp 0.15, Ki 0.005
- Acceleration: 100 rev/s²; deceleration: 20 rev/s²
- Regeneration current limit: 0.5 A
- CLOCKWISE direction

> [!CAUTION]
> A 0.5 A regeneration-current limit does not guarantee a safe DC bus in every operating condition. Verify that the power supply can absorb regenerated energy, and monitor bus voltage during the worst-case deceleration test.

### CANopen Node ID Requires Verification

The header comment in the SOLO sketch says to configure **node ID 1** in Motion Monitor, but the current source constant is:

```cpp
static constexpr uint8_t SOLO_CANOPEN_NODE_ID = 0;
```

Before using real hardware, verify this mapping against the SOLOMotorControllers 5.5.0 constructor definition and captured CAN traffic. This repository does not treat the comment and constant as a resolved, consistent configuration.

During startup and before every DISABLED-to-ENABLED transition, the sketch writes CLOCKWISE and verifies it by reading the setting back. During a normal stop, it repeatedly sends zero speed and waits 11 seconds after the last run request before switching to DISABLE. A normal PWM command that returns during this interval resumes motion immediately, but a fault never permits an automatic restart.

## Build and Upload

Open each Arduino sketch from its own directory so that auxiliary `.cpp` files in that directory are included in the build.

### ODrive or SOLO

1. Install a board core with Nano R4 support from the Arduino IDE Boards Manager.
2. Install the required libraries from Library Manager. Use SOLOMotorControllers 5.5.0 for the SOLO sketch.
3. Open the required sketch:
   - `ODRIVE_NANO_R4_CAN_HALL_v4/ODRIVE_NANO_R4_CAN_HALL_v4.ino`, or
   - `SOLO_NANO_R4_CAN_v2/SOLO_NANO_R4_CAN_v2.ino`.
4. Select the correct Nano R4 board and serial port.
5. Disconnect spindle power or remove the mechanical load, then compile and upload.
6. Open Serial Monitor at 115200 baud with the line ending set to Newline.
7. Confirm successful initialization, healthy CAN communication, and `Red: NORMAL` before testing direction at low duty.

### ESP32-C6 Test Source

1. Open `ESP32_C6_PWM_TEST/ESP32_C6_PWM_TEST.ino`.
2. Select the actual ESP32-C6 board and enable `USB CDC On Boot`.
3. Upload, then open Serial Monitor at 115200 baud with a Newline ending.
4. Enter an RPM from `0` through `15000`; the sketch maps it linearly to 0–100% duty.
5. Enter `status` to display the current setting again.

Examples:

```text
0       -> 0%
3750    -> 25%
7500    -> 50%
15000   -> 100%
```

The test source's 15,000 RPM scale differs from the 12,500 RPM ODrive scale and the 18,000 RPM SOLO scale. The physical output represents duty, not an absolute shared RPM value. Interpret it using the full-scale speed of the connected target firmware.

## Serial Commands and Diagnostics

Both Nano R4 sketches use 115200 baud and automatically print a status line approximately every 500 ms.

| Command | Action |
| --- | --- |
| `status` | Print the complete status immediately |
| `clear_errors` | Request error clearing when PWM is zero and drive communication is healthy |
| `help` | Print the available commands |

The status line includes PWM validity and duty, requested and measured speeds, Iq, the power index, bus telemetry, Blue output, drive state, protection monitors, error codes, communication state, and the Red alarm reason. The SOLO state machine also reports `STOPPED`, `RUNNING`, or `STOPPING`.

Clearing an error does not immediately authorize a restart. The firmware preserves the most recent fault for diagnosis and waits for new zero-error telemetry and safe stopped-state conditions. Record the error code and correct its cause before entering `clear_errors`.

## Protection Logic

| Alarm reason | Trigger in the current source | Recovery requirement |
| --- | --- | --- |
| `COMMUNICATION` | Eight consecutive communication or telemetry-check failures; telemetry freshness limit is 350 ms | Communication and safe-stop recovery steps must be confirmed again |
| `ODRIVE_ACTIVE_ERROR` / `SOLO_ACTIVE_ERROR` | The motor drive currently reports a nonzero error | Correct the cause, then clear at zero PWM |
| `ODRIVE_FAULT_LATCHED` / `SOLO_FAULT_LATCHED` | A drive error has been latched locally | New zero-error telemetry and safe-stop conditions must be confirmed |
| `START_FAILURE` | Duty rises to at least 5%, but the motor remains at or below 20 RPM for 2 s | Return PWM to zero and complete the stop sequence |
| `SPEED_DROP` | Target is at least 500 motor RPM and measured speed remains below 80% of target for 2 s | Return PWM to zero and complete the stop sequence |
| `POWER_INDEX` | `abs(filtered Iq) × abs(motor RPM)` remains above the configured limit for 1 s | Return PWM to zero and complete the stop sequence |

When the velocity target steps by at least 200 RPM, the speed-drop monitor waits for the motor to reach the new target again. This reduces false trips during normal acceleration. The power index is an empirical protection metric; it is not mechanical power and its unit is not watts.

## Recommended Commissioning Procedure

1. Leave motor power disconnected and check the Nano R4 Red output logic and serial log after power-up.
2. Use an oscilloscope to verify Carvera/ESP32 PWM frequency, amplitude, duty, and common ground.
3. Connect CAN only and verify 1 Mbit/s, node ID, termination, and all cyclic telemetry.
4. Rotate the motor manually and verify measured speed sign/magnitude and D3 Blue frequency.
5. Limit current, remove the tool, and run unloaded at low speed to verify direction and gear ratio.
6. Increase speed gradually while monitoring Iq, bus voltage, feedback loss, and regenerative braking.
7. Test CAN disconnection, a blocked/failed start, and a speed-drop condition separately. Verify Red alarm and shutdown behavior.
8. Connect the Carvera automatic workflow only after these tests, with an observer ready to use an independent emergency stop.

## Troubleshooting

### Communication Remains Lost

- Check for reversed CANH/CANL, missing common ground, and one 120 Ω terminator at each bus end.
- Confirm that both endpoints use 1 Mbit/s and verify the node ID.
- The ODrive must cyclically transmit Heartbeat, Encoder, Iq, Bus Voltage, and Error frames.
- Verify that D4/D5 connect to a CAN **transceiver**, not directly to the differential bus.
- Inspect consecutive failures, library errors, and the last ODrive error in the serial log.

### PWM Is Valid but the Spindle Does Not Start

- Confirm duty is above 0.2%, initialization is complete, and no fault is latched.
- Check that the ODrive can transition from IDLE to CLOSED_LOOP_CONTROL.
- Check that the SOLO startup stages completed and direction readback succeeded.
- Verify Hall calibration, current limits, bus voltage, and the emergency-stop chain.
- If `START_FAILURE` has latched, return PWM to zero and complete a safe stop first.

### The Carvera Receives No Speed Feedback

- Confirm that speed telemetry from the drive is valid. The firmware deliberately stops Blue output during a serious fault.
- Use an oscilloscope to check for a 50% duty pulse train on D3.
- Verify the 12 PPR value and the transmission-ratio configuration in the Carvera.
- Check interface voltage levels and determine whether buffering or level conversion is required.

### Duplicate Definitions When Compiling the SOLO Sketch

A newer SOLOMotorControllers release may already contain a Nano R4 backend while the local compatibility `.cpp` compiles another copy. Verify the library version, then keep either the upstream implementation or the local compatibility layer, not both.

## Checklist Before Changing Parameters

Frequently changed parameters are near the beginning of each `.ino` file. Repeat low-speed and fault tests after changing any of the following:

- `GEAR_RATIO`, `MAX_SPINDLE_RPM`, or `MAX_MOTOR_RPM`
- ODrive `DIRECTION_SIGN` or SOLO motor direction
- `SPEED_FEEDBACK_PPR`
- CAN bitrate or node ID
- Current limits, PI gains, acceleration/deceleration, or regeneration limits
- Start-failure, speed-drop, power-index, or communication timeout thresholds

This repository currently contains no automated tests, wiring schematic, reproducible Arduino CLI lockfile/configuration, or license file. Before distributing binaries or reusing the project externally, add the exact board FQBN, pinned dependency versions, hardware test records, and an appropriate license.
