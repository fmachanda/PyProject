--[[

WOW module
[affective]

Simulates weight on wheels sensor

Requires:
    XP Landing Gear

Returns:
    wow: 0 means in air, 1 means on ground

--]]

----------------------------------------------------------------
-- ONBOARD DATA ------------------------------------------------
----------------------------------------------------------------

uasDR_WOW_weight_on_wheels = create_dataref("fmuas/wow/weight_on_wheels","number") --feet

----------------------------------------------------------------
-- ACCESSED DATA -----------------------------------------------
----------------------------------------------------------------

simDR_wow = find_dataref("sim/flightmodel2/gear/on_ground")

----------------------------------------------------------------
-- SIM FUNCTIONS -----------------------------------------------
----------------------------------------------------------------

function WOW_flight_start()

	uasDR_WOW_weight_on_wheels = simDR_wow[0]

end

function WOW_before_physics()

    uasDR_WOW_weight_on_wheels = simDR_wow[0]

end

print("FMUAS-log: 0.sim.wow parsed")