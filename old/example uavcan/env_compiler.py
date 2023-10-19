import os

os.environ['CYPHAL_PATH']=f'{os.getcwd()}/custom_data_types;{os.getcwd()}/public_regulated_data_types'
os.environ['UAVCAN__NODE__ID']='42'                           # Set the local node-ID 42 (anonymous by default)
os.environ['UAVCAN__UDP__IFACE']='127.0.0.1'                  # Use Cyphal/UDP transport via localhost
os.environ['UAVCAN__SUB__TEMPERATURE_SETPOINT__ID']='2345'    # Subject "temperature_setpoint"    on ID 2345
os.environ['UAVCAN__SUB__TEMPERATURE_MEASUREMENT__ID']='2346' # Subject "temperature_measurement" on ID 2346
os.environ['UAVCAN__PUB__HEATER_VOLTAGE__ID']='2347'          # Subject "heater_voltage"          on ID 2347
os.environ['UAVCAN__SRV__LEAST_SQUARES__ID']='123'            # Service "least_squares"           on ID 123
os.environ['UAVCAN__DIAGNOSTIC__SEVERITY']='2'

os.environ['UAVCAN__SUB__VOLTAGE__ID']='2347'                 # Subject "temperature_measurement" on ID 2346
os.environ['UAVCAN__PUB__TEMPERATURE__ID']='2346'             # Subject "heater_voltage"          on ID 2347