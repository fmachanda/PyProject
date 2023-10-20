function output_cleaner()

	uasDR_AFCS_elevon1 = uasDR_AFCS_elevon1 % 360
	uasDR_AFCS_elevon2 = uasDR_AFCS_elevon2 % 360

	if uasDR_AFCS_elevon1 > 180 then uasDR_AFCS_elevon1 = uasDR_AFCS_elevon1 - 360 elseif uasDR_AFCS_elevon1 <= -180 then uasDR_AFCS_elevon1 = uasDR_AFCS_elevon1 + 360 end
	if uasDR_AFCS_elevon2 > 180 then uasDR_AFCS_elevon2 = uasDR_AFCS_elevon2 - 360 elseif uasDR_AFCS_elevon2 <= -180 then uasDR_AFCS_elevon2 = uasDR_AFCS_elevon2 + 360 end

	uasDR_AFCS_elevon1 = math.min(uasDR_AFCS_elevon1, uasSET_AFCS_elevon_max)
	uasDR_AFCS_elevon2 = math.min(uasDR_AFCS_elevon2, uasSET_AFCS_elevon_max)
	uasDR_AFCS_elevon1 = math.max(uasDR_AFCS_elevon1, uasSET_AFCS_elevon_min)
	uasDR_AFCS_elevon2 = math.max(uasDR_AFCS_elevon2, uasSET_AFCS_elevon_min)

end

function elevons_flight_start()

	uasDR_AFCS_elevon1 = 90.0
	uasDR_AFCS_elevon2 = 90.0

	uasDR_AFCS_elevon_v_yawcorr = 0.0
	uasDR_AFCS_elevon_f_pitchcorr = 0.0
	uasDR_AFCS_elevon_f_rollcorr = 0.0

end

function elevons_before_physics()

	if uasDR_AFCS_master_mode == 0 then

		uasDR_AFCS_elevon1 = 90.0
		uasDR_AFCS_elevon2 = 90.0

		uasDR_AFCS_elevon_v_yawcorr = 0.0
		uasDR_AFCS_elevon_f_pitchcorr = 0.0
		uasDR_AFCS_elevon_f_rollcorr = 0.0

		uasSET_AFCS_elevon_max = 105.0
		uasSET_AFCS_elevon_min = -15.0

	elseif uasDR_AFCS_master_mode == 1 or uasDR_AFCS_master_mode == 2 or uasDR_AFCS_master_mode == 6 or uasDR_AFCS_master_mode == 7 then

		uasDR_AFCS_elevon_v_yawcorr = 0.0 --PID(uasDR_GPSINS_yawrate, uasDR_AFCS_yawrate_cmd, 0.05, 0.002, 0.005, 4)

		uasDR_AFCS_elevon1 = uasDR_AFCS_elevon1 - uasDR_AFCS_elevon_v_yawcorr
		uasDR_AFCS_elevon2 = uasDR_AFCS_elevon2 + uasDR_AFCS_elevon_v_yawcorr

		uasSET_AFCS_elevon_max = 105.0
		uasSET_AFCS_elevon_min = 75.0

	elseif uasDR_AFCS_master_mode == 3 then

		uasDR_AFCS_elevon1 = 0.0
		uasDR_AFCS_elevon2 = 0.0

		uasSET_AFCS_elevon_max = 105.0
		uasSET_AFCS_elevon_min = -15.0

		local idontwanttodothis = true

	elseif uasDR_AFCS_master_mode == 4 then

		uasDR_AFCS_ias_gain = (((math.min(math.max(120.0 - uasDR_ADC_ias, 0.0), 60.0)) / 60.0) + 0.25) / 1.25

		uasDR_AFCS_elevon_f_pitchcorr = uasDR_AFCS_ias_gain * PID(uasDR_ADC_aoa, uasDR_AFCS_aoa_cmd, 0.65, 0.1, 0.2, 0, 3)
		uasDR_AFCS_elevon_f_rollcorr =  uasDR_AFCS_ias_gain * PID(uasDR_GPSINS_rollrate, uasDR_AFCS_rollrate_cmd, 0.11, 0.16, 0.06, 0, 4)

		uasDR_AFCS_elevon1 = -1*(uasDR_AFCS_elevon_f_pitchcorr) + 1*(uasDR_AFCS_elevon_f_rollcorr)
		uasDR_AFCS_elevon2 = -1*(uasDR_AFCS_elevon_f_pitchcorr) - 1*(uasDR_AFCS_elevon_f_rollcorr)

		uasSET_AFCS_elevon_max = 0.0
		uasSET_AFCS_elevon_min = -15.0

	elseif uasDR_AFCS_master_mode == 5 then

		uasDR_AFCS_elevon1 = 90.0
		uasDR_AFCS_elevon2 = 90.0

		uasSET_AFCS_elevon_max = 105.0
		uasSET_AFCS_elevon_min = -15.0

		local idontwanttodothis = true

	end

	output_cleaner()

end