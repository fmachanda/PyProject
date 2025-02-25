--[[

SERVOS module
[effective]

Simulates SERVOS

Requires:
    CLOCK
    AFCS

Settings:
    elevon_rate_limiter: max deg/s change in elevon angle allowed

Inputs:
    AFCS_elevon
    AFCS_elevon
    AFCS_wing_tilt

--]]

----------------------------------------------------------------
-- ONBOARD DATA ------------------------------------------------
----------------------------------------------------------------

uasDR_SERVOS_direct_mode = create_dataref("fmuas/servos/direct_mode", "number", writable) -- 1=direct mode on
uasDR_SERVOS_wing_stow_cmd = create_dataref("fmuas/servos/wing_stow", "number", writable) -- 1=stowed
uasDR_BATT_batt_cover_cmd = create_dataref("fmuas/batt_cover", "number", writable) -- 1=stowed
uasDR_BATT_batt_cover_actual = create_dataref("fmuas/batt_cover_actual", "number", writable) -- 1=stowed

function toggle_direct_mode(phase, duration)
	if phase == 0 then
		if uasDR_SERVOS_direct_mode == 0 then
			uasDR_SERVOS_direct_mode = 1
		else
			uasDR_SERVOS_direct_mode = 0
		end
	end
end

function hold_direct_mode(phase, duration)
	if phase == 1 then
		uasDR_SERVOS_direct_mode = 1
    else
		uasDR_SERVOS_direct_mode = 0
	end
end

function toggle_wing_stow(phase, duration)
    if phase == 0 then
        if simDR_radalt < .2 then
            uasDR_SERVOS_wing_stow_cmd = math.abs(uasDR_SERVOS_wing_stow_cmd - 1)
        end
    end
end

function toggle_batt_cover(phase, duration)
    if phase == 0 then
        if simDR_radalt < .2 then
            uasDR_BATT_batt_cover_cmd = math.abs(uasDR_BATT_batt_cover_cmd - 1)
        end
    end
end

cmd_handler = create_command("fmuas/commands/servos/direct_mode", "Toggle SERVOS direct mode", toggle_direct_mode)
cmd_handler = create_command("fmuas/commands/servos/temporary_direct_mode", "Temporary SERVOS direct mode", hold_direct_mode)
cmd_handler = create_command("fmuas/commands/servos/wing_stow", "Toggle wing stow", toggle_wing_stow)
cmd_handler = create_command("fmuas/commands/batt_cover", "Toggle wing stow", toggle_batt_cover)

uasSET_SERVOS_rate_limiter = create_dataref("fmuas/config/servos/elevon_rate_limiter", "number", writable)
uasSET_SERVOS_stow_rate_limiter = create_dataref("fmuas/config/servos/stow_rate_limiter", "number", writable)
uasSET_SERVOS_tilt_rate_limiter = create_dataref("fmuas/config/servos/tilt_rate_limiter", "number", writable)
uasSET_SERVOS_cover_rate_limiter = create_dataref("fmuas/config/servos/cover_rate_limiter", "number", writable)
uasSET_SERVOS_noise_gain = create_dataref("fmuas/config/servos/noise_gain", "number", writable)

uasDR_AFCS_elevon1 = create_dataref("fmuas/afcs/output/elevon1", "number", writable)
uasDR_AFCS_elevon2 = create_dataref("fmuas/afcs/output/elevon2", "number", writable)
uasDR_AFCS_rpm1 = create_dataref("fmuas/afcs/output/rpm1", "number", writable)
uasDR_AFCS_rpm2 = create_dataref("fmuas/afcs/output/rpm2", "number", writable)
uasDR_AFCS_rpm3 = create_dataref("fmuas/afcs/output/rpm3", "number", writable)
uasDR_AFCS_rpm4 = create_dataref("fmuas/afcs/output/rpm4", "number", writable)
uasDR_AFCS_wing_tilt = create_dataref("fmuas/afcs/output/wing_tilt", "number", writable)

uasDR_AFCS_elevon1 = 0.0
uasDR_AFCS_elevon2 = 0.0
uasDR_AFCS_rpm1 = 0.0
uasDR_AFCS_rpm2 = 0.0
uasDR_AFCS_rpm3 = 0.0
uasDR_AFCS_rpm4 = 0.0
uasDR_AFCS_wing_tilt = 0.0

----------------------------------------------------------------
-- ACCESSED DATA -----------------------------------------------
----------------------------------------------------------------

simDR_joystick_override = find_dataref("sim/operation/override/override_joystick")
simDR_throttle_override = find_dataref("sim/operation/override/override_throttles")

simDR_throttle1 = find_dataref("sim/flightmodel/engine/ENGN_thro_use[0]")
simDR_throttle2 = find_dataref("sim/flightmodel/engine/ENGN_thro_use[1]")
simDR_throttle3 = find_dataref("sim/flightmodel/engine/ENGN_thro_use[2]")
simDR_throttle4 = find_dataref("sim/flightmodel/engine/ENGN_thro_use[3]")

simDR_cant1 = find_dataref("sim/aircraft/prop/acf_vertcant[0]")
simDR_cant2 = find_dataref("sim/aircraft/prop/acf_vertcant[1]")
simDR_cant3 = find_dataref("sim/aircraft/prop/acf_vertcant[2]")
simDR_cant4 = find_dataref("sim/aircraft/prop/acf_vertcant[3]")

simDR_wing_tilt_actual = find_dataref("sim/flightmodel2/controls/incidence_ratio")
simDR_wing_stow_actual = find_dataref("sim/flightmodel2/controls/wingsweep_ratio")

rpm_cmd1 = create_dataref("fmuas/esc/rpm1", "number", writable)
rpm_cmd2 = create_dataref("fmuas/esc/rpm2", "number", writable)
rpm_cmd3 = create_dataref("fmuas/esc/rpm3", "number", writable)
rpm_cmd4 = create_dataref("fmuas/esc/rpm4", "number", writable)

simDR_gear = find_dataref("sim/cockpit2/controls/gear_handle_down")

simDR_elevons = find_dataref("sim/flightmodel2/wing/elements/element_incidence_increase") -- Elevon L/1 is [20], Elevon R/2 is [30]

simDR_prop_speeds = find_dataref("sim/cockpit2/engine/indicators/prop_speed_rpm")

esc_pid_kp = create_dataref("fmuas/esc/pid_kp", "number", writable)
esc_pid_ti = create_dataref("fmuas/esc/pid_ti", "number", writable)
esc_pid_td = create_dataref("fmuas/esc/pid_td", "number", writable)
esc_pid_integral = 0.0
esc_pid_prev_error = 0.0

simDR_joystick_pitch = find_dataref("sim/joystick/yoke_pitch_ratio")
simDR_joystick_roll = find_dataref("sim/joystick/yoke_roll_ratio")

simDR_axes_values_array = find_dataref("sim/joystick/joystick_axis_values")

----------------------------------------------------------------
-- INTERNAL FUNCTIONS ------------------------------------------
----------------------------------------------------------------

function servo_noiser()

    -- Adds noise to radalt data

    math.randomseed(os.clock() * 0.001)

    noise = math.random() * uasSET_SERVOS_noise_gain

end

function esc_pid(actual, setpoint)

    if setpoint<10 then
        return 0.0
    end

    error = setpoint-actual

    if math.abs(esc_pid_ti)>0.00000000000000001 then
        esc_pid_ki = esc_pid_kp/esc_pid_ti
    else
        esc_pid_ki = 0.0
    end

    esc_pid_integral = esc_pid_integral + esc_pid_ki*error*SIM_PERIOD
    esc_pid_integral = math.min(math.max(esc_pid_integral, 0.0), 1.0)

    local d = esc_pid_kp*esc_pid_td*(error - esc_pid_prev_error)/SIM_PERIOD
    esc_pid_prev_error = error

    local p = esc_pid_kp*error

    local out = p + esc_pid_integral + d

    out = math.min(math.max(out, 0.0), 1.0)

    return out
end

----------------------------------------------------------------
-- SIM FUNCTIONS -----------------------------------------------
----------------------------------------------------------------

function SERVOS_flight_start()

    esc_pid_kp = 0.001
    esc_pid_ti = 3.0
    esc_pid_td = 0.0
    esc_pid_integral = 0.0
    esc_pid_prev_error = 0.0

    uasSET_SERVOS_noise_gain = 0.001
    noise = 0.0

    uasDR_SERVOS_direct_mode = 1 --TODO
    simDR_joystick_override = 1
    simDR_throttle_override = 1

    uasSET_SERVOS_rate_limiter = 1--0.5
    uasSET_SERVOS_tilt_rate_limiter = 0.6--0.3
    uasSET_SERVOS_stow_rate_limiter = 2.5
    uasSET_SERVOS_cover_rate_limiter = 2.5

    simDR_cant3 = simDR_elevons[20] + 45.0
	simDR_cant4 = simDR_elevons[30] + 45.0
    simDR_cant1 = simDR_wing_tilt_actual * 90.0
	simDR_cant2 = simDR_wing_tilt_actual * 90.0

    uasDR_AFCS_wing_tilt = 90.0
    uasDR_AFCS_elevon1 = 90.0
    uasDR_AFCS_elevon2 = 90.0
    uasDR_SERVOS_wing_stow_cmd = 0
    uasDR_BATT_batt_cover_cmd = 0
    uasDR_BATT_batt_cover_actual = 0
    simDR_wing_stow_actual = 1.0
    simDR_wing_tilt_actual = 1.0
    uasDR_SERVOS_direct_mode = 1

    if simDR_radalt<5 then
        simDR_gear = 1.0
    else
        simDR_gear = simDR_wing_tilt_actual
    end

end

function SERVOS_after_physics()

    prev_pitch_deflection = pitch_deflection
    prev_roll_deflection = roll_deflection
    prev_tilt_cmd = tilt_cmd
    prev_stow_cmd = stow_cmd
    prev_cover_cmd = cover_cmd

    servo_noiser()

    pitch_deflection_raw = simDR_axes_values_array[simSET_pitch_axis] - 0.5
    -- pitch_deflection = 0.75+3.5*pitch_deflection_raw
    -- pitch_deflection = 0.75 + 0.5*pitch_deflection_raw
    -- pitch_deflection = -(((uasDR_AFCS_elevon1 + uasDR_AFCS_elevon2) / 2) - 45) / 60.0
    roll_deflection_raw = simDR_axes_values_array[simSET_roll_axis] - 0.5
    -- roll_deflection = 0.0
    -- roll_deflection = (uasDR_AFCS_elevon1 - uasDR_AFCS_elevon2) / 120.0
    throttle_raw = 1 - simDR_axes_values_array[simSET_throttle_axis]

    rpm_cmd1 = uasDR_AFCS_rpm1-- + (14000*throttle_raw)
    rpm_cmd2 = uasDR_AFCS_rpm2-- + (14000*throttle_raw)
    rpm_cmd3 = uasDR_AFCS_rpm3-- + (14000*throttle_raw)
    rpm_cmd4 = uasDR_AFCS_rpm4-- + (14000*throttle_raw)

    tilt_cmd = uasDR_AFCS_wing_tilt / 90.0
    -- tilt_cmd = 2*math.abs(roll_deflection_raw)

    pitch_deflection = -(((uasDR_AFCS_elevon1 + uasDR_AFCS_elevon2) / 2) - 45) / 60.0
    roll_deflection = (uasDR_AFCS_elevon1 - uasDR_AFCS_elevon2) / 120.0

    -- effective section
    pitch_deflection = math.min(pitch_deflection, prev_pitch_deflection + (uasSET_SERVOS_rate_limiter * SIM_PERIOD))
    roll_deflection = math.min(roll_deflection, prev_roll_deflection + (uasSET_SERVOS_rate_limiter * SIM_PERIOD))
    pitch_deflection = math.max(pitch_deflection, prev_pitch_deflection - (uasSET_SERVOS_rate_limiter * SIM_PERIOD))
    roll_deflection = math.max(roll_deflection, prev_roll_deflection - (uasSET_SERVOS_rate_limiter * SIM_PERIOD))

    simDR_joystick_pitch = pitch_deflection
    simDR_joystick_roll = roll_deflection

    stow_cmd = uasDR_SERVOS_wing_stow_cmd
    cover_cmd = uasDR_BATT_batt_cover_cmd

    stow_cmd = math.min(stow_cmd, prev_stow_cmd + (uasSET_SERVOS_stow_rate_limiter * SIM_PERIOD))
    tilt_cmd = math.min(tilt_cmd, prev_tilt_cmd + (uasSET_SERVOS_tilt_rate_limiter * SIM_PERIOD))
    stow_cmd = math.max(stow_cmd, prev_stow_cmd - (uasSET_SERVOS_stow_rate_limiter * SIM_PERIOD))
    tilt_cmd = math.max(tilt_cmd, prev_tilt_cmd - (uasSET_SERVOS_tilt_rate_limiter * SIM_PERIOD))
    cover_cmd = math.min(cover_cmd, prev_cover_cmd + (uasSET_SERVOS_cover_rate_limiter * SIM_PERIOD))
    cover_cmd = math.max(cover_cmd, prev_cover_cmd - (uasSET_SERVOS_cover_rate_limiter * SIM_PERIOD))

    simDR_wing_tilt_actual = tilt_cmd
    simDR_wing_stow_actual = stow_cmd
    uasDR_BATT_batt_cover_actual = cover_cmd

	simDR_cant3 = simDR_elevons[20] + 45.0
	simDR_cant4 = simDR_elevons[30] + 45.0
    simDR_cant1 = simDR_wing_tilt_actual * 90.0
	simDR_cant2 = simDR_wing_tilt_actual * 90.0

    rpm_cmd1 = math.max(math.min(rpm_cmd1, 14000), 0)
    rpm_cmd2 = math.max(math.min(rpm_cmd2, 14000), 0)
    rpm_cmd3 = math.max(math.min(rpm_cmd3, 14000), 0)
    rpm_cmd4 = math.max(math.min(rpm_cmd4, 14000), 0)

    if uasDR_SERVOS_wing_stow_cmd == 0 then
        simDR_throttle1 = esc_pid(simDR_prop_speeds[0], rpm_cmd1)
        simDR_throttle2 = esc_pid(simDR_prop_speeds[1], rpm_cmd2)
        simDR_throttle3 = esc_pid(simDR_prop_speeds[2], rpm_cmd3)
        simDR_throttle4 = esc_pid(simDR_prop_speeds[3], rpm_cmd4)
    else
        simDR_throttle1 = 0.0
        simDR_throttle2 = 0.0
        simDR_throttle3 = 0.0
        simDR_throttle4 = 0.0
    end

    if simDR_radalt<5 then
        simDR_gear = 1.0
    else
        simDR_gear = simDR_wing_tilt_actual
    end

end

print("FMUAS-log: 0.sim.servos parsed")