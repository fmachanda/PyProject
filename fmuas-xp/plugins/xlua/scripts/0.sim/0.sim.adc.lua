--[[

ADC module

Simulates ADC unit

Requires:
	pass

Settings:
	pass

Returns:
    pass

--]]

----------------------------------------------------------------
-- ONBOARD DATA ------------------------------------------------
----------------------------------------------------------------

uasDR_ADC_ias = create_dataref("fmuas/adc/ias","number")--kias
uasDR_ADC_aoa = create_dataref("fmuas/adc/aoa","number")--deg


uasSET_ADC_noise_gain = create_dataref("fmuas/config/adc/noise_gain", "number",writable)

----------------------------------------------------------------
-- ACCESSED DATA -----------------------------------------------
----------------------------------------------------------------

simDR_ias = find_dataref("sim/flightmodel/position/indicated_airspeed")
simDR_aoa = find_dataref("sim/cockpit2/gauges/indicators/AoA_pilot")

----------------------------------------------------------------
-- INTERNAL FUNCTIONS ------------------------------------------
----------------------------------------------------------------

local function adc_noiser()

	math.randomseed(os.clock() * 0.011)
	local noised_kias = simDR_ias + math.random()*uasSET_ADC_noise_gain
	uasDR_ADC_ias = noised_kias * 0.514444 --to m/s

	math.randomseed(os.clock() * 0.401)
	local noised_aoa = simDR_aoa + math.random()*uasSET_ADC_noise_gain
	uasDR_ADC_aoa = noised_aoa * 0.0174533 --to rad

end

----------------------------------------------------------------
-- SIM FUNCTIONS -----------------------------------------------
----------------------------------------------------------------

function ADC_flight_start()

	uasSET_ADC_noise_gain = 0.001

	adc_noiser()

end

function ADC_before_physics()

	adc_noiser()

end

print("FMUAS-log: 0.sim.adc parsed")