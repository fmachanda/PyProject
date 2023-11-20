function writable() end

----------------------------------------------------------------
-- ACCESSED DATA -----------------------------------------------
----------------------------------------------------------------

simDR_axes_array = find_dataref("sim/joystick/joystick_axis_assignments")

simDR_view_phi = find_dataref("sim/graphics/view/pilots_head_phi")
simDR_view_psi = find_dataref("sim/graphics/view/pilots_head_psi")
simDR_view_the = find_dataref("sim/graphics/view/pilots_head_the")
simDR_view_x = find_dataref("sim/graphics/view/pilots_head_x")
simDR_view_y = find_dataref("sim/graphics/view/pilots_head_y")
simDR_view_z = find_dataref("sim/graphics/view/pilots_head_z")

simDR_cockpit_data  = find_dataref("sim/network/dataout/data_to_screen")
old_cockpit_data = {}

simDR_brake = find_dataref("sim/cockpit2/controls/parking_brake_ratio")

simDR_fuels = find_dataref("sim/flightmodel/weight/m_fuel")
simDR_station_weights = find_dataref("sim/flightmodel/weight/m_stations")

simSET_WEIGHT_avionics = create_dataref("fmuas/config/weight/avionics_weight", "number", writable)
simSET_WEIGHT_batt = create_dataref("fmuas/config/weight/batt_weight", "number", writable)
simSET_WEIGHT_dome = create_dataref("fmuas/config/weight/dome_weight", "number", writable)
simSET_WEIGHT_fwing = create_dataref("fmuas/config/weight/fwing_weight", "number", writable)
simSET_WEIGHT_iwing = create_dataref("fmuas/config/weight/iwing_weight", "number", writable)
simSET_WEIGHT_owing = create_dataref("fmuas/config/weight/owing_weight", "number", writable)
simSET_WEIGHT_stab = create_dataref("fmuas/config/weight/stab_weight", "number", writable)

simSET_pitch_axis = create_dataref("fmuas/config/joystick/pitch_axis_index", "number")
simSET_roll_axis = create_dataref("fmuas/config/joystick/roll_axis_index", "number")
simSET_yaw_axis = create_dataref("fmuas/config/joystick/yaw_axis_index", "number")
simSET_throttle_axis = create_dataref("fmuas/config/joystick/throttle_axis_index", "number")

simCMD_pause = find_command("sim/operation/pause_toggle")
simCMD_screenshot = find_command("sim/operation/screenshot")
simDR_pause = find_dataref("sim/time/paused")

function python_change()
	simCMD_pause:once()
end

uasDR_python_running = create_dataref("fmuas/python_running", "number", python_change)

function override_python(phase, duration)
	if phase == 0 then
		if uasDR_python_running == 0 then
			uasDR_python_running = 1
		else
			uasDR_python_running = 0
		end
	end
end

cmd_handler = create_command("fmuas/commands/override_python", "Toggle override python lockout", override_python)

function image_capture(phase, duration)
	if phase == 0 then
		for i=0,199 do
			old_cockpit_data[i] = simDR_cockpit_data[i]
		end

		simDR_view_phi = 0.0
		simDR_view_psi = 0.0
		simDR_view_the = -90.0
		simDR_view_x = 0.0
		simDR_view_y = -0.034
		simDR_view_z = -0.196

		for i=0,199 do
			simDR_cockpit_data[i] = 0
		end

		simCMD_screenshot:once()
	end
end

function image_capture_reset(phase, duration)
	if phase == 0 then
		for i=0,199 do
			simDR_cockpit_data[i] = old_cockpit_data[i]
		end
	end
end

cmd_handler = create_command("fmuas/commands/image_capture", "Capture image with FLIR", image_capture)
cmd_handler = create_command("fmuas/commands/image_capture_reset", "Reset view/data after capture", image_capture_reset)

----------------------------------------------------------------
-- INTERNAL FUNCTIONS ------------------------------------------
----------------------------------------------------------------

----------------------------------------------------------------
-- LOAD MODULES ------------------------------------------------
----------------------------------------------------------------

dofile('0.sim.clock.lua')
dofile('0.sim.radalt.lua')
dofile('0.sim.gpsatt.lua')
dofile('0.sim.adc.lua')
dofile('0.sim.wow.lua')

dofile('0.sim.servos.lua')

----------------------------------------------------------------
-- SIM CALLBACKS -----------------------------------------------
----------------------------------------------------------------

function flight_start()

	uasDR_python_running = 0

	simSET_WEIGHT_avionics = 0.03
	simSET_WEIGHT_batt = 2
	simSET_WEIGHT_dome = 0.75

	simSET_WEIGHT_fwing = 0.20
	simSET_WEIGHT_iwing = 0.24
	simSET_WEIGHT_owing = 0.08
	simSET_WEIGHT_stab = 0.14

	simDR_station_weights[0] = simSET_WEIGHT_avionics
	simDR_station_weights[1] = simSET_WEIGHT_batt
	simDR_station_weights[2] = simSET_WEIGHT_batt
	simDR_station_weights[3] = simSET_WEIGHT_dome

	simDR_fuels[0] = simSET_WEIGHT_fwing
	simDR_fuels[1] = simSET_WEIGHT_iwing
	simDR_fuels[2] = simSET_WEIGHT_owing
	simDR_fuels[3] = simSET_WEIGHT_fwing
	simDR_fuels[4] = simSET_WEIGHT_iwing
	simDR_fuels[5] = simSET_WEIGHT_owing
	simDR_fuels[6] = simSET_WEIGHT_stab

	simDR_brake = 1.0

	for i=0,199 do
		old_cockpit_data[i] = simDR_cockpit_data[i]
	end

	simSET_pitch_axis = 499
	simSET_roll_axis = 499
	simSET_yaw_axis = 499
	simSET_throttle_axis = 499

	for i = 0, 499 do
		if     (simDR_axes_array[i] == 1) and (simSET_pitch_axis == 499) then simSET_pitch_axis = i
		elseif (simDR_axes_array[i] == 2) and (simSET_roll_axis == 499) then simSET_roll_axis = i
		elseif (simDR_axes_array[i] == 3) and (simSET_yaw_axis == 499) then simSET_yaw_axis = i
		elseif (simDR_axes_array[i] == 4) and (simSET_throttle_axis == 499) then simSET_throttle_axis = i end
	end

	WOW_flight_start()
    CLOCK_flight_start()
	RADALT_flight_start()
	GPSATT_flight_start()
	ADC_flight_start()

	dofile('CONFIG.lua')
	SERVOS_flight_start()

end

function before_physics()

	if (simDR_rollrate>100 or simDR_pitchrate>100 or simDR_yawrate>100) and (simDR_pause==0) then
		simCMD_pause:once()
	end

	simDR_station_weights[0] = simSET_WEIGHT_avionics
	simDR_station_weights[1] = simSET_WEIGHT_batt
	simDR_station_weights[2] = simSET_WEIGHT_batt
	simDR_station_weights[3] = simSET_WEIGHT_dome

	simDR_fuels[0] = simSET_WEIGHT_fwing
	simDR_fuels[1] = simSET_WEIGHT_iwing
	simDR_fuels[2] = simSET_WEIGHT_owing
	simDR_fuels[3] = simSET_WEIGHT_fwing
	simDR_fuels[4] = simSET_WEIGHT_iwing
	simDR_fuels[5] = simSET_WEIGHT_owing
	simDR_fuels[6] = simSET_WEIGHT_stab

	simDR_brake = 1.0

	if uasDR_python_running == 1 then

		WOW_before_physics()
		CLOCK_before_physics()
		RADALT_before_physics()
		GPSATT_before_physics()
		ADC_before_physics()

	elseif simDR_pause == 0 then
		simCMD_pause:once()
	end

end

function after_physics()

	simDR_station_weights[0] = simSET_WEIGHT_avionics
	simDR_station_weights[1] = simSET_WEIGHT_batt
	simDR_station_weights[2] = simSET_WEIGHT_batt
	simDR_station_weights[3] = simSET_WEIGHT_dome

	simDR_fuels[0] = simSET_WEIGHT_fwing
	simDR_fuels[1] = simSET_WEIGHT_iwing
	simDR_fuels[2] = simSET_WEIGHT_owing
	simDR_fuels[3] = simSET_WEIGHT_fwing
	simDR_fuels[4] = simSET_WEIGHT_iwing
	simDR_fuels[5] = simSET_WEIGHT_owing
	simDR_fuels[6] = simSET_WEIGHT_stab

	simDR_brake = 1.0

	SERVOS_after_physics()

end

print("FMUAS-log: 0.sim parsed")