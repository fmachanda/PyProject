import os
import sys

directory = os.path.dirname(os.path.realpath(__file__)) + '/..'
os.chdir(directory)

def error(msg: str) -> None:
    print("ERROR: ", msg)
    print("\nCould not setup. Please retry manually.")
    input("Press <RETURN> to exit ")
    sys.exit(1)

if sys.platform == "darwin":
    null = "&>/dev/null"
elif sys.platform == "win32":
    null = ">nul 2>nul"

if os.system(f"git --version {null}"):
    error("git not found on system PATH")

if not os.system(f"python --version"):
    python = "python"
elif not os.system(f"python3 --version"):
    python = "python3"
else:
    error("python/python3 not found on system PATH")

if os.system("git submodule update --init --recursive --remote --merge"):
    error("Could not install/update git submodules")

if os.system(f"{python} -m pip install pip --upgrade"):
    print("WARNING: Could not upgrade pip")

if os.system(f"{python} -m pip install -r requirements.txt --upgrade"):
    error("Could not install pip dependencies")

try:
    print("Generating UAVCAN files, please wait...")
    print(os.getcwd())
    sys.path.append(os.getcwd())
    for var in os.environ:
        if var.startswith('UAVCAN__'):
            os.environ.pop(var)
    os.environ['CYPHAL_PATH'] = './common/public_regulated_data_types'
    os.environ['PYCYPHAL_PATH'] = './common/pycyphal_generated'
    os.environ['UAVCAN__DIAGNOSTIC__SEVERITY'] = '2'
    import pycyphal
    import reg.udral.service.actuator.common
    import reg.udral.service.common
    import reg.udral.physics.electricity
    import reg.udral.physics.dynamics.rotation
    import reg.udral.physics.kinematics.rotation
    import reg.udral.physics.kinematics.cartesian
    import reg.udral.physics.kinematics.translation
    import reg.udral.physics.kinematics.geodetic
    import uavcan.node
    import uavcan.time
    import uavcan.si.unit.angle
    import uavcan.si.unit.length
    import uavcan.si.unit.angular_velocity
    import pycyphal.application
    import pycyphal.application.node_tracker
except Exception as e:
    error(f"Could not compile DSDL files in submodule ({e})")

print("\nSuccessful setup!")
input("Press <RETURN> to exit ")
