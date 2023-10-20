function throttles_flight_start()

	uasDR_AFCS_throttle1 = 0.0
	uasDR_AFCS_throttle2 = 0.0
	uasDR_AFCS_throttle3 = 0.0
	uasDR_AFCS_throttle4 = 0.0

	uasDR_AFCS_throttle_v_pitchcorr = 0.0
	uasDR_AFCS_throttle_v_rollcorr = 0.0

end

function throttles_before_physics()

	if uasDR_AFCS_master_mode==0 then

		uasDR_AFCS_throttle1 = 0.0
		uasDR_AFCS_throttle2 = 0.0
		uasDR_AFCS_throttle3 = 0.0
		uasDR_AFCS_throttle4 = 0.0

		uasDR_AFCS_throttle_v_pitchcorr = 0.0
		uasDR_AFCS_throttle_v_rollcorr = 0.0

	--[[elseif uasDR_AFCS_master_mode==1 then

		uasDR_AFCS_throttles_all = 0.9
		uasDR_AFCS_throttle_v_pitchcorr =PID(uasDR_GPSINS_pitchrate, uasDR_AFCS_pitchrate_cmd, 0.05, 0.002, 0.005, 5)
		uasDR_AFCS_throttle_v_rollcorr =PID(uasDR_GPSINS_rollrate, uasDR_AFCS_rollrate_cmd, 0.05, 0.002, 0.005, 6)

		uasDR_AFCS_throttle1 = uasDR_AFCS_throttle1 + uasDR_AFCS_throttle_v_pitchcorr + uasDR_AFCS_throttle_v_rollcorr + uasDR_AFCS_throttles_all
		uasDR_AFCS_throttle2 = uasDR_AFCS_throttle2 + uasDR_AFCS_throttle_v_pitchcorr - uasDR_AFCS_throttle_v_rollcorr + uasDR_AFCS_throttles_all
		uasDR_AFCS_throttle3 = uasDR_AFCS_throttle3 - uasDR_AFCS_throttle_v_pitchcorr + uasDR_AFCS_throttle_v_rollcorr + uasDR_AFCS_throttles_all
		uasDR_AFCS_throttle4 = uasDR_AFCS_throttle4 - uasDR_AFCS_throttle_v_pitchcorr - uasDR_AFCS_throttle_v_rollcorr + uasDR_AFCS_throttles_all

	elseif uasDR_AFCS_master_mode==2 or uasDR_AFCS_master_mode==6 or uasDR_AFCS_master_mode==7 then

		uasDR_AFCS_throttles_all = PID(uasDR_RADALT_vs, uasDR_AFCS_vs_cmd, 0.05, 0.002, 0.005, 8)
		uasDR_AFCS_throttle_v_pitchcorr =PID(uasDR_GPSINS_pitchrate, uasDR_AFCS_pitchrate_cmd, 0.05, 0.002, 0.005, 5)
		uasDR_AFCS_throttle_v_rollcorr =PID(uasDR_GPSINS_rollrate, uasDR_AFCS_rollrate_cmd, 0.05, 0.002, 0.005, 6)

		uasDR_AFCS_throttle1 = uasDR_AFCS_throttle1 + uasDR_AFCS_throttle_v_pitchcorr + uasDR_AFCS_throttle_v_rollcorr + uasDR_AFCS_throttles_all
		uasDR_AFCS_throttle2 = uasDR_AFCS_throttle2 + uasDR_AFCS_throttle_v_pitchcorr - uasDR_AFCS_throttle_v_rollcorr + uasDR_AFCS_throttles_all
		uasDR_AFCS_throttle3 = uasDR_AFCS_throttle3 - uasDR_AFCS_throttle_v_pitchcorr + uasDR_AFCS_throttle_v_rollcorr + uasDR_AFCS_throttles_all
		uasDR_AFCS_throttle4 = uasDR_AFCS_throttle4 - uasDR_AFCS_throttle_v_pitchcorr - uasDR_AFCS_throttle_v_rollcorr + uasDR_AFCS_throttles_all

	elseif uasDR_AFCS_master_mode==3 then

		uasDR_AFCS_throttle1 = 1.0
		uasDR_AFCS_throttle2 = 1.0
		uasDR_AFCS_throttle3 = 1.0
		uasDR_AFCS_throttle4 = 1.0

		local idontwanttodothis = true--]]

	elseif uasDR_AFCS_master_mode==4 then

		uasDR_AFCS_throttles_all = PID(uasDR_ADC_ias, uasDR_AFCS_ias_cmd, 0.04, 0.2, 0.0, 0.25, 2)

		uasDR_AFCS_throttle1 = math.min(uasDR_AFCS_throttles_all, 0.25)
		uasDR_AFCS_throttle2 = math.min(uasDR_AFCS_throttles_all, 0.25)
		uasDR_AFCS_throttle3 = math.min(uasDR_AFCS_throttles_all, 0.25)
		uasDR_AFCS_throttle4 = math.min(uasDR_AFCS_throttles_all, 0.25)

		uasDR_AFCS_throttle1 = math.max(uasDR_AFCS_throttle1, 0.02)
		uasDR_AFCS_throttle2 = math.max(uasDR_AFCS_throttle2, 0.02)
		uasDR_AFCS_throttle3 = math.max(uasDR_AFCS_throttle3, 0.02)
		uasDR_AFCS_throttle4 = math.max(uasDR_AFCS_throttle4, 0.02)

		if uasDR_AFCS_terrain_avoidance_mode == 3 then
			uasDR_AFCS_throttle1 = 1.0
			uasDR_AFCS_throttle2 = 1.0
			uasDR_AFCS_throttle3 = 1.0
			uasDR_AFCS_throttle4 = 1.0
		end

	--[[elseif uasDR_AFCS_master_mode==5 then

		uasDR_AFCS_throttle1 = 1.0
		uasDR_AFCS_throttle2 = 1.0
		uasDR_AFCS_throttle3 = 1.0
		uasDR_AFCS_throttle4 = 1.0

		local idontwanttodothis = true--]]

	end

end
