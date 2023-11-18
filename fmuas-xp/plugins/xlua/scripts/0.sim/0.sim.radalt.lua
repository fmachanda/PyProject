--[[

RADALT module
[affective]

Simulates radio altimeter

Requires:
    XP Radio Altimeter
    CLOCK

Settings:
    noise_gain: how much noise is added to radio altimeter data

Returns:
    altitude: ft
    vs: ft/s

--]]

----------------------------------------------------------------
-- ONBOARD DATA ------------------------------------------------
----------------------------------------------------------------

uasDR_RADALT_altitude = create_dataref("fmuas/radalt/altitude","number") --feet


uasSET_RADALT_noise_gain = create_dataref("fmuas/config/radalt/noise_gain","number",writable)

----------------------------------------------------------------
-- ACCESSED DATA -----------------------------------------------
----------------------------------------------------------------

simDR_radalt = find_dataref("sim/cockpit2/gauges/indicators/radio_altimeter_height_ft_pilot")

----------------------------------------------------------------
-- INTERNAL FUNCTIONS ------------------------------------------
----------------------------------------------------------------

local function radalt_noiser()

    -- Adds noise to radalt data

	math.randomseed(os.clock() * 0.001)
	local noised_altitude = simDR_radalt + math.random() * uasSET_RADALT_noise_gain
	uasDR_RADALT_altitude = noised_altitude * 0.3048 --to meters

end

----------------------------------------------------------------
-- SIM FUNCTIONS -----------------------------------------------
----------------------------------------------------------------

function RADALT_flight_start()

	uasSET_RADALT_noise_gain = 0.001

    radalt_noiser()

end

function RADALT_before_physics()

    radalt_noiser()

end

print("FMUAS-log: 0.sim.radalt parsed")