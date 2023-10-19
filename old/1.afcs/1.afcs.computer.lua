function computer_flight_start()

    uasDR_AFCS_altitude_cmd = 200.0
    uasDR_AFCS_vpath_cmd = 0.0

    uasDR_AFCS_yspeed_cmd = 0.0
    uasDR_AFCS_xspeed_cmd = 0.0

    uasDR_AFCS_yaw_cmd = 0.0
    uasDR_AFCS_yawrate_cmd = 0.0

    uasDR_AFCS_pitch_cmd = 0.0
    uasDR_AFCS_roll_cmd = 0.0

    uasDR_AFCS_aoa_cmd = 8.0
    uasDR_AFCS_rollrate_cmd = 0.0

    uasDR_AFCS_ias_cmd = 0.0

end

function computer_before_physics()

   if uasDR_AFCS_master_mode == 0 then

        uasDR_AFCS_altitude_cmd = 0.0
        uasDR_AFCS_vpath_cmd = 0.0

        uasDR_AFCS_yspeed_cmd = 0.0
        uasDR_AFCS_xspeed_cmd = 0.0

        uasDR_AFCS_yaw_cmd = uasDR_GPSINS_yaw
        uasDR_AFCS_yawrate_cmd = 0.0

        uasDR_AFCS_pitch_cmd = 0.0
        uasDR_AFCS_roll_cmd = 0.0

        uasDR_AFCS_aoa_cmd = 0.0
        uasDR_AFCS_rollrate_cmd = 0.0

    elseif uasDR_AFCS_master_mode == 1 or uasDR_AFCS_master_mode == 2 or uasDR_AFCS_master_mode == 6 or uasDR_AFCS_master_mode == 7 then

        uasDR_AFCS_altitude_cmd = 100.0
        uasDR_AFCS_vpath_cmd = 0.0

        uasDR_AFCS_yspeed_cmd = 0.0
        uasDR_AFCS_xspeed_cmd = 0.0

        uasDR_AFCS_yaw_cmd = uasDR_AFCS_yaw_cmd
        --uasDR_AFCS_yawrate_cmd = PID(uasDR_GPSINS_yaw, uasDR_AFCS_yaw_cmd, 0.05, 0, 0.005, 3)

        uasDR_AFCS_pitch_cmd = 0.0
        uasDR_AFCS_roll_cmd = 0.0

        --uasDR_AFCS_aoa_cmd = PID(uasDR_GPSINS_pitch, uasDR_AFCS_pitch_cmd, 0.05, 0, 0.005, 1)
        --uasDR_AFCS_rollrate_cmd = PID(uasDR_GPSINS_roll, uasDR_AFCS_roll_cmd, 0.05, 0, 0.005, 2)

    elseif uasDR_AFCS_master_mode == 4 then

        --debug_P, debug_I, debug_D,

        --uasDR_AFCS_altitude_cmd = 200.0

        -- TERRAIN AVOIDANCE
        if ((uasDR_RADALT_vs < -30.0) and (uasDR_RADALT_altitude < 800.0)) or ((uasDR_RADALT_vs < -10.0) and (uasDR_RADALT_altitude < 0.75 * uasDR_AFCS_altitude_cmd)) then

            uasDR_AFCS_terrain_avoidance_mode = math.min(uasDR_AFCS_terrain_avoidance_mode + 1, 3)

        elseif (uasDR_AFCS_terrain_avoidance_mode == 0) or (uasDR_RADALT_altitude > 2 * uasDR_AFCS_altitude_cmd) then

            uasDR_AFCS_terrain_avoidance_mode = 0

        end

        if uasDR_AFCS_terrain_avoidance_mode == 3 then

            uasDR_AFCS_vpath_cmd = 30.0

        else

            uasDR_AFCS_vpath_cmd = math.min(PID(uasDR_RADALT_altitude, uasDR_AFCS_altitude_cmd, 0.1, 1.0, 2.0, 0.2, 7), 4.0)

            if uasDR_RADALT_altitude > uasDR_AFCS_altitude_cmd + 200.0 then
                uasDR_AFCS_vpath_cmd = math.max(uasDR_AFCS_vpath_cmd, -6.0)
            else
                uasDR_AFCS_vpath_cmd = math.max(uasDR_AFCS_vpath_cmd, -3.0)
            end

        end

        uasDR_AFCS_aoa_cmd = math.min(PID(uasDR_ADC_vpath, uasDR_AFCS_vpath_cmd, 1.0, 2.5, 0.03, 10, 5), 16.0)
        uasDR_AFCS_aoa_cmd = math.max(uasDR_AFCS_aoa_cmd, -3.0)

        local delta_heading = uasDR_AFCS_yaw_cmd - uasDR_GPSINS_track
        if delta_heading > 180 then delta_heading = delta_heading - 360 elseif delta_heading <= -180 then delta_heading = delta_heading + 360 end
        delta_heading = -delta_heading

        uasDR_AFCS_roll_cmd = math.min(PID(delta_heading, 0.0, 1.0, 1.0, 0.0, 2.0, 8), 30.0)
        uasDR_AFCS_roll_cmd = math.max(uasDR_AFCS_roll_cmd, -30.0)

        uasDR_AFCS_rollrate_cmd = math.min(PID(uasDR_GPSINS_roll, uasDR_AFCS_roll_cmd, 5.0, 2.5, 0.01, 10, 6), 10.0)
        uasDR_AFCS_rollrate_cmd = math.max(uasDR_AFCS_rollrate_cmd, -10.0)

        uasDR_AFCS_ias_cmd = 80.0

    end

end