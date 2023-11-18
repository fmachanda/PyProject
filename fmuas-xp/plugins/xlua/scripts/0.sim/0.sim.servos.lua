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
    AFCS_wing_stow

--]]

----------------------------------------------------------------
-- ONBOARD DATA ------------------------------------------------
----------------------------------------------------------------

uasDR_SERVOS_direct_mode = create_dataref("fmuas/servos/direct_mode", "number", writable) -- 1=direct mode on

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

cmd_handler = create_command("fmuas/commands/servos/direct_mode", "Toggle SERVOS direct mode", toggle_direct_mode)
cmd_handler = create_command("fmuas/commands/servos/temporary_direct_mode", "Temporary SERVOS direct mode", hold_direct_mode)

uasSET_SERVOS_rate_limiter = create_dataref("fmuas/config/servos/elevon_rate_limiter", "number", writable) --deg/s
uasSET_SERVOS_noise_gain = create_dataref("fmuas/config/servos/noise_gain", "number", writable)

uasDR_AFCS_elevon1 = create_dataref("fmuas/afcs/output/elevon1", "number", writable)
uasDR_AFCS_elevon2 = create_dataref("fmuas/afcs/output/elevon2", "number", writable)
uasDR_AFCS_throttle1 = create_dataref("fmuas/afcs/output/throttle1", "number", writable)
uasDR_AFCS_throttle2 = create_dataref("fmuas/afcs/output/throttle2", "number", writable)
uasDR_AFCS_throttle3 = create_dataref("fmuas/afcs/output/throttle3", "number", writable)
uasDR_AFCS_throttle4 = create_dataref("fmuas/afcs/output/throttle4", "number", writable)
uasDR_AFCS_wing_tilt = create_dataref("fmuas/afcs/output/wing_tilt", "number", writable)
uasDR_AFCS_wing_stow = create_dataref("fmuas/afcs/output/wing_stow", "number", writable)

uasDR_AFCS_elevon1 = 0.0
uasDR_AFCS_elevon2 = 0.0
uasDR_AFCS_throttle1 = 0.0
uasDR_AFCS_throttle2 = 0.0
uasDR_AFCS_throttle3 = 0.0
uasDR_AFCS_throttle4 = 0.0
uasDR_AFCS_wing_tilt = 0.0
uasDR_AFCS_wing_stow = 0.0

----------------------------------------------------------------
-- ACCESSED DATA -----------------------------------------------
----------------------------------------------------------------

simDR_joystick_override = find_dataref("sim/operation/override/override_joystick")

simDR_throttle1 = find_dataref("sim/cockpit2/engine/actuators/throttle_ratio[0]")
simDR_throttle2 = find_dataref("sim/cockpit2/engine/actuators/throttle_ratio[1]")
simDR_throttle3 = find_dataref("sim/cockpit2/engine/actuators/throttle_ratio[2]")
simDR_throttle4 = find_dataref("sim/cockpit2/engine/actuators/throttle_ratio[3]")

simDR_cant1 = find_dataref("sim/aircraft/prop/acf_vertcant[0]")
simDR_cant2 = find_dataref("sim/aircraft/prop/acf_vertcant[1]")
simDR_cant3 = find_dataref("sim/aircraft/prop/acf_vertcant[2]")
simDR_cant4 = find_dataref("sim/aircraft/prop/acf_vertcant[3]")

simDR_wing_stow = find_dataref("sim/cockpit2/controls/wingsweep_ratio") -- RATIO: 0 is flight / 1 is stowed
-- simDR_wing_stow_actual = find_dataref("sim/flightmodel2/controls/wingsweep_ratio")
simDR_wing_tilt = find_dataref("sim/cockpit2/controls/incidence_ratio") -- RATIO: 0 is flight / 1 is VTOL
simDR_wing_tilt_actual = find_dataref("sim/flightmodel2/controls/incidence_ratio")

simDR_elevons = find_dataref("sim/flightmodel2/wing/elements/element_incidence_increase") -- Elevon L/1 is [20], Elevon R/2 is [30]

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

----------------------------------------------------------------
-- SIM FUNCTIONS -----------------------------------------------
----------------------------------------------------------------

function SERVOS_flight_start()

    uasSET_SERVOS_noise_gain = 0.001
    noise = 0.0

    uasDR_SERVOS_direct_mode = 0
    simDR_joystick_override = 1

    uasSET_SERVOS_rate_limiter = 0.5

    simDR_cant3 = simDR_elevons[20] + 45.0
	simDR_cant4 = simDR_elevons[30] + 45.0
    simDR_cant1 = simDR_wing_tilt_actual * 90.0
	simDR_cant2 = simDR_wing_tilt_actual * 90.0

end

function SERVOS_after_physics()

    prev_pitch_deflection = pitch_deflection
    prev_roll_deflection = roll_deflection

    servo_noiser()

    if uasDR_SERVOS_direct_mode == 1 then
        pitch_deflection_raw = simDR_axes_values_array[simSET_pitch_axis]
        pitch_deflection = ((pitch_deflection_raw - 0.5) / 2) + 0.75
        -- pitch_deflection = -(((uasDR_AFCS_elevon1 + uasDR_AFCS_elevon2) / 2) - 45) / 60.0
        roll_deflection_raw = simDR_axes_values_array[simSET_roll_axis]
        roll_deflection = (roll_deflection_raw - 0.5) / 24
        -- roll_deflection = (uasDR_AFCS_elevon1 - uasDR_AFCS_elevon2) / 120.0
    else

        simDR_throttle1 = uasDR_AFCS_throttle1
        simDR_throttle2 = uasDR_AFCS_throttle2
        simDR_throttle3 = uasDR_AFCS_throttle3
        simDR_throttle4 = uasDR_AFCS_throttle4

        simDR_wing_tilt = uasDR_AFCS_wing_tilt / 90.0
        simDR_wing_stow = uasDR_AFCS_wing_stow / 90.0

        pitch_deflection = -(((uasDR_AFCS_elevon1 + uasDR_AFCS_elevon2) / 2) - 45) / 60.0
        roll_deflection = (uasDR_AFCS_elevon1 - uasDR_AFCS_elevon2) / 120.0

    end

    pitch_deflection = math.min(pitch_deflection, prev_pitch_deflection + (uasSET_SERVOS_rate_limiter * SIM_PERIOD))
    roll_deflection = math.min(roll_deflection, prev_roll_deflection + (uasSET_SERVOS_rate_limiter * SIM_PERIOD))
    pitch_deflection = math.max(pitch_deflection, prev_pitch_deflection - (uasSET_SERVOS_rate_limiter * SIM_PERIOD))
    roll_deflection = math.max(roll_deflection, prev_roll_deflection - (uasSET_SERVOS_rate_limiter * SIM_PERIOD))

    simDR_joystick_pitch = pitch_deflection
    simDR_joystick_roll = roll_deflection

	simDR_cant3 = simDR_elevons[20] + 45.0
	simDR_cant4 = simDR_elevons[30] + 45.0
    simDR_cant1 = simDR_wing_tilt_actual * 90.0
	simDR_cant2 = simDR_wing_tilt_actual * 90.0

end

print("FMUAS-log: 0.sim.servos parsed")