# Codex maintenance context

## Scope

This repository is a collection of alternative Arduino spindle-interface experiments, not one combined application. Never compile multiple sketch directories together or silently designate a production variant. Preserve the safety gates and variant-specific electrical behavior.

## Source map

- Hall ODrive candidates: `ODRIVE_NANO_R4_CAN_v3_HALL/ODRIVE_NANO_R4_CAN_v3_HALL.ino` and `ODRIVE_NANO_R4_CAN_HALL_v4/ODRIVE_NANO_R4_CAN_HALL_v4.ino`.
- Incremental ODrive lineage: `ODRIVE_NANO_R4_CAN_v1/`, `v2/`, `v3/`; v3 targets ODrive firmware 0.6.12 and adds `index_search`.
- Legacy/general ODrive: `ODRIVE_NANO_R4_CAN/`; its Red output semantics differ from newer high-impedance-normal variants.
- SOLO: `SOLO_NANO_R4_CAN_v2/SOLO_NANO_R4_CAN_v2.ino` plus `SOLO_NANO_R4_CAN_v2_compat.cpp` for SOLOMotorControllers 5.5.0/Nano R4 native CAN compatibility.
- Analog interface: `CARVERA_ESCON_INTERFACEBOARD/CARVERA_ESCON_INTERFACEBOARD.ino`.
- Signal generator: `ESP32_C6_PWM_TEST/ESP32_C6_PWM_TEST.ino`.
- ODrive configuration and motion exercises: `Odrive.py`, `odrivetest.py`.
- Non-source evidence: `error.txt` and ignored `.arduino-build-*` directories.

## Shared data flow

Carvera PWM on Nano D2 -> duty filtering/mapping -> controller velocity command -> controller telemetry/fault monitors -> D3 speed-feedback pulse train and D6 alarm. Native Nano R4 CAN is D4 TX/D5 RX through an external transceiver.

## Invariants and hazards

- Prove direction at low speed before changing `DIRECTION_SIGN` or allowing normal operation.
- Do not remove zero-command/state gates around `clear_errors`, `index_search`, or `motor_identification`.
- Preserve safe-stop, communication timeout, speed-drop, start-failure, bus/error, and power monitoring unless a requested change explicitly redesigns safety behavior.
- Verify current, voltage, regen, ratio, encoder, node ID, and controller-firmware assumptions against the selected hardware.
- ESP32-C6 GPIO is 3.3 V-only. Its test sketch intentionally starts with a 7,500 rpm-equivalent PWM command.
- `odrivetest.py` causes physical motion up to 80 turns/s. `Odrive.py` saves and reboots the drive and initially disables CAN.

## Build and validation

The only repository build evidence is historical Nano R4 output for `SOLO_NANO_R4_CAN_v2` using FQBN `arduino:renesas_uno:nanor4`:

```powershell
arduino-cli compile --fqbn arduino:renesas_uno:nanor4 SOLO_NANO_R4_CAN_v2
```

There is no CI or automated test suite. Validate the exact chosen sketch first and record the board core, controller firmware, library version, and FQBN. Hardware validation must begin without a cutter/workpiece, with an independent E-stop and a low-speed direction test. Compilation and the existing serial log are not end-to-end proof.

## Known uncertainty

- No canonical firmware or production release is declared.
- Exact Carvera signal levels and CAN transceiver model are absent.
- SOLO documentation says CANopen node ID 1 while source passes node argument 0; confirm library addressing before changing it.
- No license is present.

## Change discipline

Read the selected `.ino` header before editing. Keep changes scoped to one hardware branch unless cross-variant behavior is explicitly requested. Do not edit generated build directories or the captured log. Update both readmes when user-visible wiring, commands, dependencies, defaults, or safety behavior changes.
