--[[

GPS/INS module
[affective]

Simulates pre-integrated GPS/INS unit (no drift corrections, etc.)

Requires:
	XP AHARS
	XP Indicators
	CLOCK

Settings:
	rotational_noise_gain: how much noise to add to rotational data
	positional_noise_gain: how much noise to add to positional data

Returns:
    pitch: degrees
	roll: degrees
	yaw: degrees magnetic
	pitchrate: degrees per second
	rollrate: degrees per second
	yawrate: degrees per second
	track: degrees magnetic
	gspeed: knots
	yspeed: knots, relative to UAS lateral axis
	xspeed: knots, relative to UAS longitudinal axis
	latitude: GPS
	longitude: GPS

--]]

----------------------------------------------------------------
-- ONBOARD DATA ------------------------------------------------
----------------------------------------------------------------

uasDR_GPSATT_att_qx = create_dataref("fmuas/att/attitude_quaternion_x", "number")
uasDR_GPSATT_att_qy = create_dataref("fmuas/att/attitude_quaternion_y", "number")
uasDR_GPSATT_att_qz = create_dataref("fmuas/att/attitude_quaternion_z", "number")
uasDR_GPSATT_att_qw = create_dataref("fmuas/att/attitude_quaternion_w", "number")

uasDR_GPSATT_pitchrate = create_dataref("fmuas/att/pitchrate", "number")
uasDR_GPSATT_rollrate = create_dataref("fmuas/att/rollrate", "number")
uasDR_GPSATT_yawrate = create_dataref("fmuas/att/yawrate", "number")

uasDR_GPSATT_vn = create_dataref("fmuas/gps/vn", "number")
uasDR_GPSATT_ve = create_dataref("fmuas/gps/ve", "number")
uasDR_GPSATT_vd = create_dataref("fmuas/gps/vd", "number")

uasDR_GPSATT_latitude = create_dataref("fmuas/gps/latitude", "number")
uasDR_GPSATT_longitude = create_dataref("fmuas/gps/longitude", "number")
uasDR_GPSATT_altitude = create_dataref("fmuas/gps/altitude", "number")


uasSET_GPSATT_rotational_noise_gain = create_dataref("fmuas/config/gpsatt/rot_noise_gain", "number", writable)
uasSET_GPSATT_positional_noise_gain = create_dataref("fmuas/config/gpsatt/pos_noise_gain", "number", writable)

----------------------------------------------------------------
-- ACCESSED DATA -----------------------------------------------
----------------------------------------------------------------

simDR_att_quat = find_dataref("sim/flightmodel/position/q")

simDR_rollrate = find_dataref("sim/flightmodel/position/Prad")
simDR_pitchrate = find_dataref("sim/flightmodel/position/Qrad")
simDR_yawrate = find_dataref("sim/flightmodel/position/Rrad")

simDR_vn = find_dataref("sim/flightmodel/position/local_vz")
simDR_ve = find_dataref("sim/flightmodel/position/local_vx")
simDR_vd_neg = find_dataref("sim/flightmodel/position/local_vy")

simDR_latitude = find_dataref("sim/flightmodel/position/latitude")
simDR_longitude = find_dataref("sim/flightmodel/position/longitude")
simDR_gpsaltitude = find_dataref("sim/flightmodel/position/elevation")

----------------------------------------------------------------
-- INTERNAL FUNCTIONS ------------------------------------------
----------------------------------------------------------------

local function gpsatt_noiser()

	-- Adds noise to GPS/INS data

	math.randomseed(os.clock() * 0.001)
	uasDR_GPSATT_att_qw = simDR_att_quat[0] + math.random()*uasSET_GPSATT_rotational_noise_gain

	math.randomseed(os.clock() * 0.101)
	uasDR_GPSATT_att_qx = simDR_att_quat[1] + math.random()*uasSET_GPSATT_rotational_noise_gain

	math.randomseed(os.clock() * 0.011)
	uasDR_GPSATT_att_qy = simDR_att_quat[2] + math.random()*uasSET_GPSATT_rotational_noise_gain

	math.randomseed(os.clock() * 0.002)
	uasDR_GPSATT_att_qz = simDR_att_quat[3] + math.random()*uasSET_GPSATT_rotational_noise_gain


	math.randomseed(os.clock() * 0.201)
	uasDR_GPSATT_rollrate = simDR_rollrate + math.random()*uasSET_GPSATT_rotational_noise_gain

	math.randomseed(os.clock() * 0.041)
	uasDR_GPSATT_pitchrate = simDR_pitchrate + math.random()*uasSET_GPSATT_rotational_noise_gain

	math.randomseed(os.clock() * 0.301)
	uasDR_GPSATT_yawrate = simDR_yawrate + math.random()*uasSET_GPSATT_rotational_noise_gain


	math.randomseed(os.clock() * 0.071)
	uasDR_GPSATT_vn = -simDR_vn + math.random()*uasSET_GPSATT_positional_noise_gain

	math.randomseed(os.clock() * 0.121)
	uasDR_GPSATT_ve = simDR_ve + math.random()*uasSET_GPSATT_positional_noise_gain

	math.randomseed(os.clock() * 0.031)
	uasDR_GPSATT_vd = -simDR_vd_neg + math.random()*uasSET_GPSATT_positional_noise_gain

	math.randomseed(os.clock() * 0.091)
	local noised_lat = simDR_latitude + math.random()*uasSET_GPSATT_positional_noise_gain
	uasDR_GPSATT_latitude = noised_lat * math.pi / 180 -- To rads

	math.randomseed(os.clock() * 0.102)
	local noised_long = simDR_longitude + math.random()*uasSET_GPSATT_positional_noise_gain
	uasDR_GPSATT_longitude = noised_long * math.pi / 180 -- To rads

	math.randomseed(os.clock() * 0.006)
	local noised_altitude = simDR_gpsaltitude + math.random()*uasSET_GPSATT_positional_noise_gain
	uasDR_GPSATT_altitude = noised_altitude -- In m

end

----------------------------------------------------------------
-- SIM FUNCTIONS -----------------------------------------------
----------------------------------------------------------------

function GPSATT_flight_start()

	uasSET_GPSATT_rotational_noise_gain = 0.001
	uasSET_GPSATT_positional_noise_gain = 0.001

	gpsatt_noiser()

end

function GPSATT_before_physics()

	gpsatt_noiser()

end

print("FMUAS-log: 0.sim.gpsins parsed")