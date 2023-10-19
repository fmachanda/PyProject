function proc_flight_start()

	if uasSET_start == 0 then
		uasDR_AFCS_master_mode = 0
		uasDR_AFCS_wing_stow = 1.0
		uasDR_AFCS_wing_tilt = 1.0
	else
		uasDR_AFCS_master_mode = 4
		uasDR_AFCS_wing_stow = 0.0
		uasDR_AFCS_wing_tilt = 0.0
	end

end

function proc_before_physics()

	if uasDR_AFCS_master_mode == 0 then

		-- ground

		uasDR_AFCS_wing_stow = 1.0
		uasDR_AFCS_wing_tilt = 1.0

	elseif uasDR_AFCS_master_mode == 1 then

		-- takeoff

		uasDR_AFCS_wing_stow = 1.0
		uasDR_AFCS_wing_tilt = 1.0

	elseif uasDR_AFCS_master_mode == 2 then

		-- hover

		uasDR_AFCS_wing_stow = 0.0
		uasDR_AFCS_wing_tilt = 1.0

	elseif uasDR_AFCS_master_mode == 3 then

		-- transition forward

		uasDR_AFCS_wing_stow = 0.0
		uasDR_AFCS_wing_tilt = 0.0

	elseif uasDR_AFCS_master_mode == 4 then

		-- flight

		uasDR_AFCS_wing_stow = 0.0
		uasDR_AFCS_wing_tilt = 0.0

	elseif uasDR_AFCS_master_mode == 5 then

		-- transition back

		uasDR_AFCS_wing_stow = 0.0
		uasDR_AFCS_wing_tilt = 1.0

	elseif uasDR_AFCS_master_mode == 6 then

		-- hover

		uasDR_AFCS_wing_stow = 1.0
		uasDR_AFCS_wing_tilt = 1.0

		--[[
		if safe_landing_bool == true then
			-- before landing checklist
			inc_master_mode()
		end
		--]]

	elseif uasDR_AFCS_master_mode == 7 then

		-- landing

		uasDR_AFCS_wing_stow = 1.0
		uasDR_AFCS_wing_tilt = 1.0

	end

end