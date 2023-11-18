--[[

CLOCK module
[affective]

Simulates clock

Requires:
    none

Settings:
    sum_res: how many frames to sum (int)

Returns:
    step: same as SIM_PERIOD
    sum: summed SIM_PERIODS over last sum_res number of frames
    time: time in seconds from boot

--]]

----------------------------------------------------------------
-- ONBOARD DATA ------------------------------------------------
----------------------------------------------------------------

uasDR_CLOCK_time = create_dataref("fmuas/clock/time","number") --secs

----------------------------------------------------------------
-- INTERNAL FUNCTIONS ------------------------------------------
----------------------------------------------------------------

local function calc_time()

    --Increments CLOCK time by SIM_PERIOD

    uasDR_CLOCK_time = uasDR_CLOCK_time + SIM_PERIOD

end

----------------------------------------------------------------
-- SIM FUNCTIONS -----------------------------------------------
----------------------------------------------------------------

function CLOCK_flight_start()

    uasDR_CLOCK_time = 0.0

end

function CLOCK_before_physics()

    calc_time()

end

print("FMUAS-log: 0.sim.clock parsed")