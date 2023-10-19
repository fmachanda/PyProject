@echo off

set CYPHAL_PATH=./data_types/custom_data_types;./data_types/public_regulated_data_types

set UAVCAN__NODE__ID=42                           & rem Set the local node-ID 42 (anonymous by default)
set UAVCAN__UDP__IFACE=127.0.0.1                  & rem Use Cyphal/UDP transport via localhost
set UAVCAN__SUB__TEMPERATURE_SETPOINT__ID=2345    & rem Subject "temperature_setpoint"    on ID 2345
set UAVCAN__SUB__TEMPERATURE_MEASUREMENT__ID=2346 & rem Subject "temperature_measurement" on ID 2346
set UAVCAN__PUB__HEATER_VOLTAGE__ID=2347          & rem Subject "heater_voltage"          on ID 2347
set UAVCAN__SRV__LEAST_SQUARES__ID=123            & rem Service "least_squares"           on ID 123
set UAVCAN__DIAGNOSTIC__SEVERITY=2                & rem This is optional to enable logging via Cyphal

python -c "print('\n')"
python demo_app.py                                & rem Run the application!