function yd_flight_start()

	uasDR_AFCS_yaw_out = 0.0

end

function yd_before_physics()

	if uasDR_AFCS_master_mode == 0 then

		uasDR_AFCS_yaw_out = 0.0

	elseif uasDR_AFCS_master_mode == 3 then

		local idontwanttodothis = true

	elseif uasDR_AFCS_master_mode == 4 then

		uasDR_AFCS_yaw_out = math.min(PID(uasDR_ADC_slip, 0.0, -0.15, -0.1, 0.6, 0, 1), 5.0)
		uasDR_AFCS_yaw_out = math.max(uasDR_AFCS_yaw_out, -1.0)

	elseif uasDR_AFCS_master_mode == 5 then

		local idontwanttodothis = true

	end

end