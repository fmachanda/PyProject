@echo off

set CYPHAL_PATH=./data_types/custom_data_types;./data_types/public_regulated_data_types

set UAVCAN__NODE__ID=43                           & rem Set the local node-ID 43 (anonymous by default)
set UAVCAN__UDP__IFACE=127.0.0.1                  & rem Use Cyphal/UDP transport via localhost
set UAVCAN__PUB__TEMPERATURE__ID=2346             & rem Subject "temperature_measurement" on ID 2346
set UAVCAN__SUB__VOLTAGE__ID=2347                 & rem Subject "heater_voltage"          on ID 2347

set MODEL__ENVIRONMENT__TEMPERATURE=250

python -c "print('\n')"
python plant.py                          & rem Run the application!