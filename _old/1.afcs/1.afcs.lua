--[[
function writable() end

debug_P = create_dataref("fmuas/tune_pid/input/p", "number", writable)
debug_I = create_dataref("fmuas/tune_pid/input/i", "number", writable)
debug_D = create_dataref("fmuas/tune_pid/input/d", "number", writable)
debug_set = create_dataref("fmuas/tune_pid/input/_set", "number", writable)

debug_P = 0.0
debug_I = 0.0
debug_D = 0.0
debug_set = 0.0

uasDR_AFCS_master_mode = create_dataref("fmuas/afcs/master_mode", "number", writable) -- 0:'ground' 1:'takeoff' 2:'hover_to' 3:'transition_to' 4:'flight' 5:'transition_land' 6:'hover_land' 7:'land'

function inc_master_mode(phase, duration)
	if phase == 0 then
		if uasDR_AFCS_master_mode == 7 then
			uasDR_AFCS_master_mode = 0
		else
			uasDR_AFCS_master_mode = uasDR_AFCS_master_mode + 1
		end
	end
end

function dec_master_mode(phase, duration)
	if phase == 0 then
		if uasDR_AFCS_master_mode == 0 then
			uasDR_AFCS_master_mode = 0
		else
			uasDR_AFCS_master_mode = uasDR_AFCS_master_mode - 1
		end
	end
end

cmd_handler = create_command("fmuas/commands/afcs/inc_master_mode", "step AFCS computer master mode", inc_master_mode)
cmd_handler = create_command("fmuas/commands/afcs/dec_master_mode", "unstep AFCS computer master mode", dec_master_mode)

uasDR_AFCS_elevon1 = create_dataref("fmuas/afcs/output/elevon1", "number")
uasDR_AFCS_elevon2 = create_dataref("fmuas/afcs/output/elevon2", "number")
uasDR_AFCS_yaw_out = create_dataref("fmuas/afcs/output/yaw", "number")
uasDR_AFCS_throttle1 = create_dataref("fmuas/afcs/output/throttle1", "number")
uasDR_AFCS_throttle2 = create_dataref("fmuas/afcs/output/throttle2", "number")
uasDR_AFCS_throttle3 = create_dataref("fmuas/afcs/output/throttle3", "number")
uasDR_AFCS_throttle4 = create_dataref("fmuas/afcs/output/throttle4", "number")
uasDR_AFCS_wing_tilt = create_dataref("fmuas/afcs/output/wing_tilt", "number")
uasDR_AFCS_wing_stow = create_dataref("fmuas/afcs/output/wing_stow", "number")

uasSET_AFCS_elevon_min = create_dataref("fmuas/config/afcs/elevon_min", "number", writable)
uasSET_AFCS_elevon_max = create_dataref("fmuas/config/afcs/elevon_max", "number", writable)

uasDR_AFCS_pitch_cmd = create_dataref("fmuas/afcs/setpoints/pitch_cmd", "number", writable)
uasDR_AFCS_roll_cmd = create_dataref("fmuas/afcs/setpoints/roll_cmd", "number", writable)
uasDR_AFCS_yaw_cmd = create_dataref("fmuas/afcs/setpoints/yaw_cmd", "number", writable)
uasDR_AFCS_aoa_cmd = create_dataref("fmuas/afcs/setpoints/aoa_cmd", "number", writable)
uasDR_AFCS_pitchrate_cmd = create_dataref("fmuas/afcs/setpoints/pitchrate_cmd", "number", writable)
uasDR_AFCS_rollrate_cmd = create_dataref("fmuas/afcs/setpoints/rollrate_cmd", "number", writable)
uasDR_AFCS_yawrate_cmd = create_dataref("fmuas/afcs/setpoints/yawrate_cmd", "number", writable)
uasDR_AFCS_yspeed_cmd = create_dataref("fmuas/afcs/setpoints/yspeed_cmd", "number", writable)
uasDR_AFCS_xspeed_cmd = create_dataref("fmuas/afcs/setpoints/xspeed_cmd", "number", writable)
uasDR_AFCS_vpath_cmd = create_dataref("fmuas/afcs/setpoints/vpath_cmd", "number", writable)
uasDR_AFCS_altitude_cmd = create_dataref("fmuas/afcs/setpoints/altitude_cmd", "number", writable)
uasDR_AFCS_ias_cmd = create_dataref("fmuas/afcs/setpoints/ias_cmd", "number", writable)

uasDR_AFCS_elevon_v_yawcorr = create_dataref("fmuas/afcs/pids/vtol/con_yawcorr", "number")
uasDR_AFCS_elevon_f_pitchcorr = create_dataref("fmuas/afcs/pids/flight/con_pitchcorr", "number")
uasDR_AFCS_elevon_f_rollcorr = create_dataref("fmuas/afcs/pids/flight/con_rollcorr", "number")
uasDR_AFCS_throttle_v_pitchcorr = create_dataref("fmuas/afcs/pids/flight/throttles_pitchcorr", "number")
uasDR_AFCS_throttle_v_rollcorr = create_dataref("fmuas/afcs/pids/flight/throttles_rollcorr", "number")
uasDR_AFCS_throttles_all = create_dataref("fmuas/afcs/pids/throttles_all", "number", writable)
uasDR_AFCS_throttles_all = 0.20

uasDR_AFCS_ias_gain = create_dataref("fmuas/afcs/ias_gain", "number")

uasDR_AFCS_terrain_avoidance_mode = create_dataref("fmuas/afcs/uasDR_AFCS_terrain_avoidance_mode", "number")

----------------------------------------------------------------
-- LOAD MODULES ------------------------------------------------
----------------------------------------------------------------
dofile('1.afcs.pid.lua')
dofile('1.afcs.computer.lua')
dofile('1.afcs.elevons.lua')
dofile('1.afcs.throttles.lua')
dofile('1.afcs.yawdamper.lua')
dofile('1.afcs.proc.lua')

function flight_start()

	uasSET_start = find_dataref("fmuas/config/start")

	uasDR_AFCS_terrain_avoidance_mode = 0

	uasDR_GPSINS_pitch = find_dataref("fmuas/gpsins/pitch")
	uasDR_GPSINS_roll = find_dataref("fmuas/gpsins/roll")
	uasDR_GPSINS_yaw = find_dataref("fmuas/gpsins/yaw")
	uasDR_GPSINS_pitchrate = find_dataref("fmuas/gpsins/pitchrate")
	uasDR_GPSINS_rollrate = find_dataref("fmuas/gpsins/rollrate")
	uasDR_GPSINS_yawrate = find_dataref("fmuas/gpsins/yawrate")
	uasDR_GPSINS_yspeed = find_dataref("fmuas/gpsins/yspeed")
	uasDR_GPSINS_xspeed = find_dataref("fmuas/gpsins/xspeed")
	uasDR_GPSINS_latitude = find_dataref("fmuas/gpsins/latitude")
	uasDR_GPSINS_longitude = find_dataref("fmuas/gpsins/longitude")
	uasDR_GPSINS_track = find_dataref("fmuas/gpsins/track") --deg mag
	uasDR_GPSINS_gspeed = find_dataref("fmuas/gpsins/gspeed") --kts

	uasDR_ADC_slip = find_dataref("fmuas/adc/slip") --deg
	uasDR_ADC_vpath = find_dataref("fmuas/adc/vpath") --deg
	uasDR_ADC_aoa = find_dataref("fmuas/adc/aoa") --deg
	uasDR_ADC_ias = find_dataref("fmuas/adc/ias") --kias

	uasDR_RADALT_altitude = find_dataref("fmuas/radalt/altitude") --kts
	uasDR_RADALT_vs = find_dataref("fmuas/radalt/vs") --kts

	uasDR_CLOCK_step = find_dataref("fmuas/clock/step") --secs
	uasDR_CLOCK_sum = find_dataref("fmuas/clock/sum") --secs
	uasDR_CLOCK_time = find_dataref("fmuas/clock/time") --secs
	uasSET_CLOCK_sum_res = find_dataref("fmuas/config/clock/sum_res")

	uasDR_WOW_weight_on_wheels = find_dataref("fmuas/wow/weight_on_wheels") --feet

	------------------------------------------------------------------------------------------------

	computer_flight_start()

	proc_flight_start()
	elevons_flight_start()
	throttles_flight_start()
	yd_flight_start()

end

function before_physics()

	proc_before_physics()

	computer_before_physics()

	elevons_before_physics()
	throttles_before_physics()
	yd_before_physics()

end--]]