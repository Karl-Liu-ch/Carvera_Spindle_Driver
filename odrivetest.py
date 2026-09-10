import time

TARGET_VEL = 80.0
RAMP_STEP = 5.0
RAMP_DELAY = 0.2

print("========================================")
print("INITIAL STATUS")
print("========================================")

print("State:", odrv0.axis0.current_state)
print("Errors:", hex(odrv0.axis0.active_errors))
print("Disarm:", hex(odrv0.axis0.disarm_reason))

print("\n--- Encoder ---")
print("CPR:", odrv0.inc_encoder0.config.cpr)
print("Pole pairs:", odrv0.axis0.config.motor.pole_pairs)
print("Observed scale:",
      odrv0.axis0.observed_encoder_scale_factor)

print("\n--- Controller ---")
print("Vel gain:",
      odrv0.axis0.controller.config.vel_gain)

print("Vel integrator gain:",
      odrv0.axis0.controller.config.vel_integrator_gain)

print("Vel integrator:",
      odrv0.axis0.controller.vel_integrator_torque)

print("\n--- Current ---")
print("Iq set:",
      odrv0.axis0.motor.foc.Iq_setpoint)

print("Iq:",
      odrv0.axis0.motor.foc.Iq_measured)

print("Id:",
      odrv0.axis0.motor.foc.Id_measured)


# ============================================================
# Save filter values
# ============================================================

old_current_filter = \
    odrv0.axis0.motor.foc.I_measured_report_filter_k

old_ibus_filter = \
    odrv0.ibus_report_filter_k

print("\nOriginal current filter:",
      old_current_filter)

print("Original Ibus filter:",
      old_ibus_filter)


# ============================================================
# Apply strong reporting filter
# ============================================================

odrv0.axis0.motor.foc.I_measured_report_filter_k = 0.01

odrv0.ibus_report_filter_k = 0.01

print("\nNew current filter:",
      odrv0.axis0.motor.foc.I_measured_report_filter_k)

print("New Ibus filter:",
      odrv0.ibus_report_filter_k)


# ============================================================
# Configure velocity control
# ============================================================

odrv0.axis0.controller.config.control_mode = 2
odrv0.axis0.controller.config.input_mode = 1

odrv0.axis0.controller.input_vel = 0.0


# ============================================================
# Enter closed loop
# ============================================================

print("\n========================================")
print("ENTER CLOSED LOOP")
print("========================================")

odrv0.axis0.requested_state = 8

time.sleep(1)

print("State:", odrv0.axis0.current_state)
print("Errors:", hex(odrv0.axis0.active_errors))

if odrv0.axis0.current_state != 8:
    raise RuntimeError("Could not enter closed loop")


# ============================================================
# Ramp to 80 turn/s
# ============================================================

print("\n========================================")
print("RAMP UP")
print("========================================")

cmd = 0.0

while cmd < TARGET_VEL:

    cmd = min(
        TARGET_VEL,
        cmd + RAMP_STEP
    )

    odrv0.axis0.controller.input_vel = cmd

    time.sleep(RAMP_DELAY)

    print(
        f"cmd={cmd:5.1f} | "
        f"vel={odrv0.axis0.vel_estimate:7.2f} | "
        f"Iq_set={odrv0.axis0.motor.foc.Iq_setpoint:7.3f}"
    )

    if odrv0.axis0.active_errors != 0:
        print(
            "ERROR:",
            hex(odrv0.axis0.active_errors)
        )
        break


# ============================================================
# Wait for filters to settle
# ============================================================

print("\n========================================")
print("SETTLING 5 SECONDS")
print("========================================")

for i in range(10):

    print(
        f"vel={odrv0.axis0.vel_estimate:7.2f} | "
        f"Iq_set={odrv0.axis0.motor.foc.Iq_setpoint:7.3f} | "
        f"Iq={odrv0.axis0.motor.foc.Iq_measured:7.3f} | "
        f"Id={odrv0.axis0.motor.foc.Id_measured:7.3f}"
    )

    time.sleep(0.5)


# ============================================================
# Stable measurement
# ============================================================

print("\n========================================")
print("STABLE 80 TURN/S")
print("========================================")

for i in range(50):

    vel = odrv0.axis0.vel_estimate

    iq_set = \
        odrv0.axis0.motor.foc.Iq_setpoint

    iq = \
        odrv0.axis0.motor.foc.Iq_measured

    id_current = \
        odrv0.axis0.motor.foc.Id_measured

    vel_int = \
        odrv0.axis0.controller.vel_integrator_torque

    vbus = odrv0.vbus_voltage

    ibus = odrv0.ibus

    pelec = \
        odrv0.axis0.motor.electrical_power

    pmech = \
        odrv0.axis0.motor.mechanical_power

    print(
        f"vel={vel:7.2f} | "
        f"Iq_set={iq_set:7.3f} | "
        f"Iq={iq:7.3f} | "
        f"Id={id_current:7.3f} | "
        f"vel_int={vel_int:7.4f} | "
        f"Ibus={ibus:7.3f} | "
        f"Pelec={pelec:7.1f} | "
        f"Pmech={pmech:7.1f}"
    )

    if odrv0.axis0.active_errors != 0:
        print(
            "ERROR:",
            hex(odrv0.axis0.active_errors)
        )
        break

    time.sleep(0.2)


# ============================================================
# Ramp down
# ============================================================

print("\n========================================")
print("RAMP DOWN")
print("========================================")

cmd = TARGET_VEL

while cmd > 0:

    cmd = max(
        0,
        cmd - RAMP_STEP
    )

    odrv0.axis0.controller.input_vel = cmd

    time.sleep(RAMP_DELAY)

    print(
        f"cmd={cmd:5.1f} | "
        f"vel={odrv0.axis0.vel_estimate:7.2f}"
    )


# ============================================================
# Stop
# ============================================================

odrv0.axis0.controller.input_vel = 0

time.sleep(1)

odrv0.axis0.requested_state = 1

time.sleep(0.5)


# ============================================================
# Restore filters
# ============================================================

odrv0.axis0.motor.foc.I_measured_report_filter_k = \
    old_current_filter

odrv0.ibus_report_filter_k = \
    old_ibus_filter


print("\n========================================")
print("FINISHED")
print("========================================")

print("Final state:",
      odrv0.axis0.current_state)

print("Errors:",
      hex(odrv0.axis0.active_errors))

print("Current filter restored:",
      odrv0.axis0.motor.foc.I_measured_report_filter_k)

print("Ibus filter restored:",
      odrv0.ibus_report_filter_k)

# odrv0.axis0.requested_state = AxisState.ENCODER_INDEX_SEARCH