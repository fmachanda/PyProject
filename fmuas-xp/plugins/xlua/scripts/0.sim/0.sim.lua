XLuaReloadOnFlightChange()

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

uasDR_CAM_pitch_actual = create_dataref("fmuas/camera/pitch_actual", "number")
uasDR_CAM_roll_actual = create_dataref("fmuas/camera/roll_actual", "number")
uasDR_CAM_pitch_cmd = create_dataref("fmuas/camera/pitch", "number", writable)
uasDR_CAM_roll_cmd = create_dataref("fmuas/camera/roll", "number", writable)

uasDR_CAM_eff_pitch = create_dataref("fmuas/camera/effective_pitch", "number", writable)
uasDR_CAM_eff_yaw = create_dataref("fmuas/camera/effective_yaw", "number", writable)

uasDR_CAM_rate_limit = create_dataref("fmuas/camera/rate_limit", "number", writable)

simDR_brake = find_dataref("sim/cockpit2/controls/parking_brake_ratio")

simDR_fuels = find_dataref("sim/flightmodel/weight/m_fuel")
simDR_station_weights = find_dataref("sim/flightmodel/weight/m_stations")

simDR_WEIGHT_empty = find_dataref("sim/aircraft/weight/acf_m_empty")

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

ROLL = find_dataref("sim/flightmodel/position/phi")
HDG = find_dataref("sim/flightmodel/position/psi")


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

function rp_to_py()
	Rd = uasDR_CAM_roll_actual
	Pd = uasDR_CAM_pitch_actual

    Rd = math.pi * Rd / 180
    Pd = math.pi * Pd / 180

    Pc = math.asin(math.sin(Pd) * math.cos(Rd))

	if math.cos(Pc)~=0.0 then
		Yc = math.acos(math.max(math.min(math.cos(Pd) / math.cos(Pc), 1), -1))
		if ((-math.pi<Rd)and(Rd<0)) or ((math.pi<Rd)and(Rd<2*math.pi)) then
			Yc = -Yc
		end
	else
        Yc = 0.0
	end

	Pc = 180 * Pc / math.pi
	Yc = 180 * Yc / math.pi

    uasDR_CAM_eff_pitch = Pc
	uasDR_CAM_eff_yaw = Yc
end

cmd_handler = create_command("fmuas/commands/override_python", "Toggle override python lockout", override_python)

function image_capture(phase, duration)
	if phase == 0 then
		print("Commanding FLIR image")
		for i=0,199 do
			old_cockpit_data[i] = simDR_cockpit_data[i]
		end

		rp_to_py()

		calc_phi = (uasDR_CAM_roll_actual*(1 - math.abs(uasDR_CAM_eff_yaw/90)) + uasDR_CAM_eff_yaw)
		if (calc_phi<-90) or (calc_phi>90) then
			simDR_view_phi = (180+calc_phi)%360
		else
			simDR_view_phi = calc_phi
		end
		simDR_view_psi = uasDR_CAM_eff_yaw
		simDR_view_the = uasDR_CAM_eff_pitch
		simDR_view_x = 0.0
		simDR_view_y = 0.0
		simDR_view_z = -0.22

		for i=0,199 do
			simDR_cockpit_data[i] = 0
		end

		simCMD_screenshot:once()
		uasCMD_image_reset:once()
		print("Successful FLIR image")
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
uasCMD_image_reset = create_command("fmuas/commands/_image_capture_reset", "Reset view/data after capture", image_capture_reset)

uasDR_flir_view_on = create_dataref("fmuas/view/camera_view_on", "number")

function flir_view(phase, duration)
	if phase == 0 then
		uasDR_flir_view_on = math.abs(uasDR_flir_view_on-1)
	end
end

cmd_handler = create_command("fmuas/commands/toggle_camera_view", "Turn on/off FLIR view", flir_view)

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

	simDR_WEIGHT_empty = 0.907185

	uasDR_python_running = 0
	uasDR_flir_view_on = 0

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

	uasDR_CAM_rate_limit = 60.0

	uasDR_CAM_pitch_actual = 180.0
	uasDR_CAM_pitch_cmd = 180.0
	uasDR_CAM_roll_actual = 0.0
	uasDR_CAM_roll_cmd = 0.0

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

	uasDR_CAM_pitch_actual = math.max(math.min(uasDR_CAM_pitch_cmd, uasDR_CAM_pitch_actual+(uasDR_CAM_rate_limit*SIM_PERIOD)), uasDR_CAM_pitch_actual-(uasDR_CAM_rate_limit*SIM_PERIOD))
	uasDR_CAM_roll_actual = math.max(math.min(uasDR_CAM_roll_cmd, uasDR_CAM_roll_actual+(uasDR_CAM_rate_limit*SIM_PERIOD)), uasDR_CAM_roll_actual-(uasDR_CAM_rate_limit*SIM_PERIOD))

	rp_to_py()

	if uasDR_flir_view_on==1 then
		simDR_view_phi = (uasDR_CAM_roll_actual*(1 - math.abs(uasDR_CAM_eff_yaw/90)) + uasDR_CAM_eff_yaw)
		simDR_view_psi = uasDR_CAM_eff_yaw
		simDR_view_the = uasDR_CAM_eff_pitch
		simDR_view_x = 0.0
		simDR_view_y = 0.0
		simDR_view_z = -0.22
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