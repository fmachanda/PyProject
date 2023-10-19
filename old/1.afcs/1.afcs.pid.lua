-- VTOL
	-- altitude > vvi > throttles
	-- yspeed > pitch > pitchrate > throttley
	-- xspeed > roll > rollrate > throttlex
	-- yaw > yawrate > elevons

-- FLIGHT
	-- altitude > vvi > pitch > pitchrate > elevons
	-- heading > turnrate > roll > rollrate > elevons
	-- speed > throttles

	-- slip >> rudder ?

out_P = create_dataref("fmuas/tune_pid/ouput/p", "number")
out_I = create_dataref("fmuas/tune_pid/ouput/i", "number")
out_D = create_dataref("fmuas/tune_pid/ouput/d", "number")
debug_index = create_dataref("fmuas/tune_pid/index", "number", writable)
debug_index = 8

PID_integral = {	0.0;	-- DEBUG
					0.0; 	-- 1  (F) yaw damper	from 	slip
					0.0; 	-- 2  (F) throttle		from 	speed
					0.0; 	-- 3  (F) elevon pitch 	from	aoa
					0.0; 	-- 4  (F) elevons roll	from	rollrate
					0.0; 	-- 5  (F) aoa			from	vpath
					0.0; 	-- 6  (F) rollrate		from	roll
					0.0; 	-- 7  (F) vpath			from	altitude
					0.0; 	-- 8  (F) roll			from	heading
}

PID_preverror = {	0.0;	-- DEBUG
					0.0; 	-- 1  (F) yaw damper	from 	slip
					0.0; 	-- 2  (F) throttle		from 	speed
					0.0; 	-- 3  (F) elevon pitch 	from	aoa
					0.0; 	-- 4  (F) elevons roll	from	rollrate
					0.0; 	-- 5  (F) aoa			from	vpath
					0.0; 	-- 6  (F) rollrate		from	roll
					0.0; 	-- 7  (F) vpath			from	altitude
					0.0; 	-- 8  (F) roll			from	heading
}

--debug_PID_index=0

function PID(value, setpoint, PID_kp, PID_ti, PID_td, PID_integral_limit, PID_index)

	local PID_ki

	if PID_ti ~= 0.0 then
		PID_ki = PID_kp / PID_ti
	else
		PID_ki = 0.0
	end

	local error = setpoint - value

	local proportional = error

	PID_integral[PID_index] = PID_integral[PID_index] + (PID_ki * error * uasDR_CLOCK_step)

	if PID_integral_limit ~= 0 then
		PID_integral[PID_index] = math.min(PID_integral[PID_index],  (PID_integral_limit / PID_kp))
		PID_integral[PID_index] = math.max(PID_integral[PID_index], -(PID_integral_limit / PID_kp))
	end

	local derivative = PID_td * (error - PID_preverror[PID_index]) / uasDR_CLOCK_step

	if PID_index == debug_index then
		out_P = proportional * PID_kp
		out_I = PID_integral[PID_index] * PID_kp
		out_D = derivative * PID_kp
	end

	local output = (proportional * PID_kp) + (PID_integral[PID_index] * PID_kp) + (derivative * PID_kp)
	PID_preverror[PID_index] = error
	return output
end

--[[
debug_PID_setpoint=create_dataref("fmuas/debug/PID_setpoint", "number", writable)
debug_PID_value=create_dataref("fmuas/debug/PID_value", "number")
debug_PID_integral=create_dataref("fmuas/debug/PID_integral", "number")
debug_PID_preverror=create_dataref("fmuas/debug/PID_preverror", "number")
debug_PID_kp=create_dataref("fmuas/debug/PID_kp", "number", writable)
debug_PID_ki=create_dataref("fmuas/debug/PID_ki", "number", writable)
debug_PID_kd=create_dataref("fmuas/debug/PID_kd", "number", writable)
debug_PID_output=create_dataref("fmuas/debug/PID_output", "number")

function debug_PID_RESET()
	debug_PID_setpoint=0.0
	debug_PID_value=0.0
	debug_PID_integral=0.0
	PID_integral[6]=0.0
	debug_PID_preverror=0.0
	PID_preverror[6]=0.0
	debug_PID_kp=0.05
	debug_PID_ki=0.001
	debug_PID_kd=0.005
end

debug_PID_RESET_handler=create_command("fmuas/debug_PID_RESET", "reset the debugger pid", debug_PID_RESET)--]]