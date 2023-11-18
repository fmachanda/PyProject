-- Set desired configurations at flight start
uasSET_start = create_dataref("fmuas/config/start","number") -- 0=ground 1=air

if (uasDR_WOW_weight_on_wheels == 1) or (uasDR_RADALT_altitude < 1.0) then

    -- Start on ground
    uasSET_start = 0

    simDR_wing_tilt = 1.0
    simDR_wing_tilt_actual = 1.0
    simDR_wing_stow = 1.0
    simDR_wing_stow_actual = 1.0

    print("FMUAS: CONFIG initialized (ground)")

else

    -- Start in air
    uasSET_start = 1

    simDR_wing_tilt = 0.0
    simDR_wing_tilt_actual = 0.0
    simDR_wing_stow = 0.0
    simDR_wing_stow_actual = 0.0
    simDR_elevons[20] = 0.0
    simDR_elevons[30] = 0.0

    print("FMUAS: CONFIG initialized (air)")

end