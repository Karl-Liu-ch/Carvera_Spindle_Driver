# Carvera Spindle Driver Experiments

Arduino firmware and ODrive bench scripts for translating a Carvera spindle-speed command into commands for several spindle-drive arrangements. The repository contains alternative, mutually exclusive implementations; it does **not** identify one sketch as production firmware.

## Safety first

This project can command a high-speed spindle and can expose hazardous electrical, mechanical, and regenerative-energy conditions.

- Remove the cutter and workpiece for commissioning, secure the spindle, and keep an independent emergency stop available.
- Disconnect power before rewiring. Do not use firmware fault handling as a replacement for external safety circuits.
- Verify motor direction at very low speed before increasing the command or changing `DIRECTION_SIGN`.
- Confirm controller ratings, current limits, bus-voltage limits, braking/regen absorption, and motor/encoder configuration for the actual hardware.
- Use a suitable CAN transceiver, common ground, and 120-ohm termination at both physical ends of the bus.
- Level-shift or isolate incompatible signal domains. ESP32-C6 GPIO is 3.3 V and is not 5 V tolerant.

The constants in these sketches are project assumptions, not certified equipment ratings.

## Repository contents

| Path | Purpose |
| --- | --- |
| `ODRIVE_NANO_R4_CAN_v3_HALL/` | Nano R4 to ODrive S1 CANSimple firmware using Hall feedback; targets ODrive firmware 0.6.11. |
| `ODRIVE_NANO_R4_CAN_HALL_v4/` | Alternate Hall-based ODrive branch with additional start-failure monitoring. |
| `ODRIVE_NANO_R4_CAN_v3/` | Incremental-encoder ODrive branch targeting firmware 0.6.12; includes the USB `index_search` command. |
| `ODRIVE_NANO_R4_CAN_v1/`, `v2/`, `ODRIVE_NANO_R4_CAN/` | Earlier ODrive variants retained for comparison and hardware-specific use. |
| `SOLO_NANO_R4_CAN_v2/` | Nano R4 to SOLO PICO over CANopen, including a Nano R4 compatibility translation unit. |
| `CARVERA_ESCON_INTERFACEBOARD/` | Nano R4 PWM-to-DAC interface for an ESCON-style analog speed input. |
| `ESP32_C6_PWM_TEST/` | ESP32-C6 command-signal generator for supervised bench testing. |
| `Odrive.py` | Commands intended for an `odrivetool` session to configure an ODrive S1 for an AS5047P SPI encoder. |
| `odrivetest.py` | Interactive `odrivetool` bench exercise that physically accelerates the motor. |
| `error.txt` | Captured serial log from one Hall/ODrive run; sample evidence, not a test suite or certification. |

Generated `.arduino-build-*` directories are ignored build artifacts, not source. There is no automated test suite or CI configuration in this repository.

## Common Nano R4 wiring

The current ODrive and SOLO sketches document this shared arrangement:

| Signal | Nano R4 pin | Notes |
| --- | --- | --- |
| Carvera spindle PWM command | D2 | Approximately 1 kHz input. |
| Blue speed feedback | D3 | Generated as 12 pulses per spindle revolution. |
| Native CAN TX / RX | D4 / D5 | Connect through the correct CAN transceiver. |
| Red alarm | D6 | Newer variants drive LOW for fault and otherwise use high impedance. The earliest ODrive variant differs. |

Connect all required signal grounds. Verify the Carvera and transceiver electrical specifications before connecting them; those specifications and the exact transceiver model are not defined here.

## Firmware behavior

The main controller sketches measure and filter the Carvera PWM duty cycle, map it through a configured spindle-to-motor ratio, command the selected controller, and synthesize speed feedback. Newer variants monitor controller communication, controller errors, failed starts, sustained speed drop, and estimated power loading before asserting the alarm and requesting a safe stop.

Important defaults vary by branch:

- ODrive Hall v3: 1.635 ratio, 12,500 rpm spindle maximum, 8,400 rpm motor maximum, CAN node 0 at 1 Mbit/s.
- SOLO v2: 1.635 ratio, 16,000 rpm spindle maximum, 9,700 rpm motor maximum, 8.2 A current limit, and 0.5 A regenerative limit.
- The ODrive and SOLO controllers must be commissioned and saved with the matching motor, feedback, control, and CAN settings described at the top of the selected `.ino` file.

Common USB serial commands in the newer controller variants include `status`, `clear_errors`, and `help`. SOLO v2 also exposes `motor_identification`; incremental ODrive v3 exposes `index_search`. Motion-related commands are intentionally gated on a zero spindle command and suitable controller state.

## Building

Install Arduino IDE 2 or Arduino CLI, then install the board core and libraries required by the one selected sketch. Open or compile one sketch directory at a time.

Nano R4 example (the repository contains historical build metadata for this target):

```powershell
arduino-cli compile --fqbn arduino:renesas_uno:nanor4 SOLO_NANO_R4_CAN_v2
```

Dependencies:

- ODrive/Nano sketches use `Arduino_CAN.h` and the Renesas core's `pwm.h`.
- SOLO v2 expects the SOLOMotorControllers library; its compatibility file documents version 5.5.0 and includes the library's private implementation files to enable Nano R4 native CAN.
- ESP32-C6 testing requires an ESP32 Arduino board package and **USB CDC On Boot** enabled. The sketch deliberately starts at a 7,500 rpm-equivalent command, so connect it only to a safe measurement setup until that behavior is understood or changed.

No upload port is encoded in the repository. Select the board and port for the connected device, compile first, inspect warnings, and upload only after checking the wiring and configuration comments in the chosen source file.

## ODrive scripts

`Odrive.py` assumes an active `odrivetool` session with `odrv0` already defined. It saves configuration and reboots the drive. Its default configuration disables CAN, so CAN firmware requires a subsequent matching CAN configuration.

`odrivetest.py` also assumes `odrv0`; it changes controller/filter state and ramps the motor to 80 turns/s. Treat it as a supervised bench procedure, not a standalone command-line utility.

## Known limitations

- No canonical production firmware, release process, automated tests, or license is declared.
- Controller firmware and feedback hardware differ between branches; do not copy configuration blindly between them.
- The relationship between SOLO's commissioned CANopen node ID 1 and the source's node argument 0 may reflect library indexing, but is not proven in this repository.
- The captured log demonstrates one apparently stable run near 12,000 spindle rpm; it does not validate other operating points or hardware.

See [README_CN.md](README_CN.md) for the Chinese documentation and [codex_context.md](codex_context.md) for a concise maintenance map.
