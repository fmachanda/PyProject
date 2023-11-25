"""UAV Main

This script connects to simulated hardware through realistic UAVCAN and 
MAVLINK protocols. The script recieves data from various sensors and 
publishes desired servo and throttle settings. A method to connect with 
a GCS over MAVLINK is provided. The UAV is designed to be autonomous.

This script uses values from the common/config.ini file.

See https://github.com/fmachanda/fmuas-main for more details.
"""

import argparse
import asyncio
import logging
import math
import os
import sys
from configparser import ConfigParser
from functools import wraps

import numpy as np

os.chdir(os.path.dirname(os.path.realpath(__file__)) + '/..')
sys.path.append(os.getcwd())
os.environ['CYPHAL_PATH'] = './common/public_regulated_data_types'
os.environ['PYCYPHAL_PATH'] = './common/pycyphal_generated'
os.environ['UAVCAN__DIAGNOSTIC__SEVERITY'] = '2'
os.environ['MAVLINK20'] = '1'
os.environ['MAVLINK_DIALECT'] = 'common'

filehandler = logging.FileHandler('uav/uav.log', mode='w')
filehandler.setLevel(logging.INFO)
filehandler.setFormatter(logging.Formatter('(%(asctime)s %(name)s) %(levelname)s:%(message)s'))
streamhandler = logging.StreamHandler()
streamhandler.setLevel(logging.INFO)
logging.basicConfig(format='%(name)s %(levelname)s:%(message)s', level=logging.DEBUG, handlers=[filehandler, streamhandler])

logging.warning("Generating UAVCAN files, please wait...")

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
from pymavlink import mavutil

import common.grapher as grapher
import common.image_processor as img
from common.pid import PID
from common.state_manager import GlobalState as g
from common.angles import quaternion_to_euler, euler_to_quaternion, gps_angles, calc_dyaw

m = mavutil.mavlink

for name in ['pymavlink', 'pycyphal', 'pydsdl', 'nunavut', 'matplotlib']:
    logging.getLogger(name).setLevel(logging.WARNING)
filehandler.setLevel(logging.DEBUG)

os.system('cls' if os.name == 'nt' else 'clear')

DEFAULT_FREQ = 60
HEARTBEAT_TIMEOUT = 2.0

RPM_TO_RADS = math.pi/30

_DEBUG_SKIP = 2 # TODO: remove this!!

system_ids = []


def async_loop_decorator(close=True):
    """Provide decorator to gracefully loop coroutines"""
    def decorator(func):
        @wraps(func) # Preserve metadata like func.__name__
        async def wrapper(self: 'MainIO | Processor | ImageProcessor | Controller | Navigator', *args, **kwargs):
            try:
                while not self.main.stop.is_set():
                    try:
                        await asyncio.sleep(0)
                        await func(self, *args, **kwargs)
                    except asyncio.exceptions.CancelledError:
                        self.main.stop.set()
                        raise
                    except Exception as e:
                        logging.error(f"Error in {func.__name__}: {e}")
                        raise # TODO
            except KeyboardInterrupt:
                self.main.stop.set()
                raise
            finally:
                if close:
                    self.close()
                else:
                    logging.debug(f"Closing {func.__name__}")
        return wrapper
    return decorator


class GlobalRx:
    """Store sensor data."""
    class Time:
        """Store clock data."""
        def __init__(self) -> None:
            self.time = 0.0
            self._last_time = 0.0
            self.dt = 0.0
        
        def dump(self, msg: uavcan.time.SynchronizedTimestamp_1) -> None:
            """Store data from a message."""
            self._last_time = self.time
            self.time = msg.microsecond
            self.dt = self.time - self._last_time

    class Att:
        """Store attitude data."""
        def __init__(self) -> None:
            self.time = 0.0
            self.pitch = 0.0
            self.pitchspeed = 0.0
            self.roll = 0.0
            self.rollspeed = 0.0
            self.yaw = 0.0
            self.yawspeed = 0.0
            self._last_time = 0.0
            self.dt = 0.0
            # self._last_pitch = 0.0
            # self._last_pitchspeed = 0.0
            # self._last_roll = 0.0
            # self._last_rollspeed = 0.0
            # self._last_yaw = 0.0
            # self._last_yawspeed = 0.0

        def dump(self, msg: reg.udral.physics.kinematics.cartesian.StateVarTs_0) -> None:
            """Store data from a message."""
            self._last_time = self.time
            # self._last_pitch = self.pitch
            # self._last_pitchspeed = self.pitchspeed
            # self._last_roll = self.roll
            # self._last_rollspeed = self.rollspeed
            # self._last_yaw = self.yaw
            # self._last_yawspeed = self.yawspeed

            self.time = msg.timestamp.microsecond
            self.roll, self.pitch, self.yaw = quaternion_to_euler(msg.value.pose.value.orientation.wxyz)

            if self.yaw < 0:
                self.yaw += 2*math.pi

            self.rollspeed, self.pitchspeed, self.yawspeed = msg.value.twist.value.angular.radian_per_second
            # TODO: *self.speeds = msg.value.twise.value.linear.meter_per_second
            self.dt = self.time - self._last_time

    class Alt:
        """Store altimeter data."""
        def __init__(self) -> None:
            self.time = 0.0
            self.altitude = 0.0
            self.vs = 0.0

            self._last_time = 0.0
            self.dt = 0.0
            self._last_altitude = 0.0
            self._last_vs = 0.0

        def dump(self, msg: uavcan.si.unit.length.WideScalar_1, time: 'GlobalRx.Time.time') -> None:
            """Store data from a message."""
            self._last_time = self.time
            self._last_altitude = self.altitude
            self._last_vs = self.vs

            self.time = time
            self.altitude = msg.meter

            try:
                self.vs = (self.altitude - self._last_altitude) / (self.time - self._last_time)
            except ZeroDivisionError:
                self.vs = self._last_vs

            self.dt = self.time - self._last_time

    class Gps:
        """Store GPS data."""
        def __init__(self) -> None:
            self.time = 0.0
            self.latitude = 41.688306
            self.longitude = -83.716114
            self.altitude = 0.0
            self.nspeed = 0.0
            self.espeed = 0.0
            self.dspeed = 0.0
            
            self.xspeed = 0.0
            self.yspeed = 0.0

            self._last_time = 0.0
            self.dt = 0.0
            # self._last_latitude = 0.0
            # self._last_longitude = 0.0
            # self._last_altitude = 0.0
            # self._last_nspeed = 0.0
            # self._last_espeed = 0.0
            # self._last_dspeed = 0.0

        def dump(self, msg: reg.udral.physics.kinematics.geodetic.PointStateVarTs_0, heading: 'GlobalRx.Att.yaw') -> None:
            """Store data from a message."""
            self._last_time = self.time
            # self._last_latitude = self.latitude
            # self._last_longitude = self.longitude
            # self._last_altitude = self.altitude
            # self._last_nspeed = self.nspeed
            # self._last_espeed = self.espeed
            # self._last_dspeed = self.dspeed

            self.time = msg.timestamp.microsecond
            self.latitude = math.degrees(msg.value.position.value.latitude)
            self.longitude = math.degrees(msg.value.position.value.longitude)
            self.altitude = msg.value.position.value.altitude
            self.nspeed, self.espeed, self.dspeed = msg.value.velocity.value.meter_per_second

            self.yspeed = self.nspeed*math.cos(heading) + self.espeed*math.sin(heading)
            self.xspeed = self.nspeed*math.sin(heading) + self.espeed*math.cos(heading)

            self.dt = self.time - self._last_time

    class Ias:
        """Store airspeed data."""
        def __init__(self) -> None:
            self.time = 0.0
            self.ias = 0.0

            self._last_time = 0.0
            self.dt = 0.0
            # self._last_ias = 0.0

        def dump(self, msg: reg.udral.physics.kinematics.translation.LinearTs_0) -> None:
            """Store data from a message."""
            self._last_time = self.time
            # self._last_ias = self.ias
            
            self.time = msg.timestamp.microsecond
            self.ias = msg.value.velocity.meter_per_second

            self.dt = self.time - self._last_time

    class Aoa:
        """Store AOA data."""
        def __init__(self) -> None:
            self.time = 0.0
            self.aoa = 0.0

            self._last_time = 0.0
            self.dt = 0.0
            # self._last_aoa = 0.0

        def dump(self, msg: uavcan.si.unit.angle.Scalar_1, time: 'GlobalRx.Time.time') -> None:
            """Store data from a message."""
            self._last_time = self.time
            # self._last_aoa = self.aoa
            
            self.time = time
            self.aoa = msg.radian

            self.dt = self.time - self._last_time

    class Cam:
        """Store CAM data."""
        def __init__(self) -> None:
            self.time = 0.0
            self.xdp = 0.0
            self.ydp = 0.0

            self._last_time = 0.0
            self.dt = 0.0
            # self._last_xdp = 0.0
            # self._last_ydp = 0.0

        def dump(self, xdp: float, ydp: float, time: 'GlobalRx.Time.time') -> None:
            """Store data from a message."""
            self._last_time = self.time
            # self._last_aoa = self.aoa
            
            self.time = time
            self.xdp = xdp
            self.ydp = ydp

            self.dt = self.time - self._last_time

    def __init__(self) -> None:
        self.time = GlobalRx.Time()
        self.att =  GlobalRx.Att()
        self.alt =  GlobalRx.Alt()
        self.gps =  GlobalRx.Gps()
        self.ias =  GlobalRx.Ias()
        self.aoa =  GlobalRx.Aoa()
        self.cam =  GlobalRx.Cam()


class GlobalTx:
    """Store control data before publishing."""
    def __init__(self) -> None:
        self._servo_readiness = reg.udral.service.common.Readiness_0(
            reg.udral.service.common.Readiness_0.ENGAGED
        )

        self._esc_readiness = reg.udral.service.common.Readiness_0(
            reg.udral.service.common.Readiness_0.ENGAGED
        )

        self._elevon1 = reg.udral.physics.dynamics.rotation.Planar_0(
            reg.udral.physics.kinematics.rotation.Planar_0(
                uavcan.si.unit.angle.Scalar_1(0.0)
            ),
            # torque
        )

        self._elevon2 = reg.udral.physics.dynamics.rotation.Planar_0(
            reg.udral.physics.kinematics.rotation.Planar_0(
                uavcan.si.unit.angle.Scalar_1(0.0)
            ),
            # torque
        )
        
        self._tilt = reg.udral.physics.dynamics.rotation.Planar_0(
            reg.udral.physics.kinematics.rotation.Planar_0(
                uavcan.si.unit.angle.Scalar_1(0.0)
            ),
            # torque
        )
        
        self._stow = reg.udral.physics.dynamics.rotation.Planar_0(
            reg.udral.physics.kinematics.rotation.Planar_0(
                uavcan.si.unit.angle.Scalar_1(0.0)
            ),
            # torque
        )
        
        self._esc1 = reg.udral.physics.dynamics.rotation.Planar_0(
            reg.udral.physics.kinematics.rotation.Planar_0(
                angular_velocity=uavcan.si.unit.angular_velocity.Scalar_1(0.0)
            ),
            # torque
        )
        
        self._esc2 = reg.udral.physics.dynamics.rotation.Planar_0(
            reg.udral.physics.kinematics.rotation.Planar_0(
                angular_velocity=uavcan.si.unit.angular_velocity.Scalar_1(0.0)
            ),
            # torque
        )
        
        self._esc3 = reg.udral.physics.dynamics.rotation.Planar_0(
            reg.udral.physics.kinematics.rotation.Planar_0(
                angular_velocity=uavcan.si.unit.angular_velocity.Scalar_1(0.0)
            ),
            # torque
        )
        
        self._esc4 = reg.udral.physics.dynamics.rotation.Planar_0(
            reg.udral.physics.kinematics.rotation.Planar_0(
                angular_velocity=uavcan.si.unit.angular_velocity.Scalar_1(0.0)
            ),
            # torque
        )

    # Properties to prevent having to construct CYPHAL message each time
    @property
    def servo_readiness(self) -> reg.udral.service.common.Readiness_0:
        return self._servo_readiness
    
    @property
    def esc_readiness(self) -> reg.udral.service.common.Readiness_0:
        return self._servo_readiness
    
    @property
    def elevon1(self) -> reg.udral.physics.dynamics.rotation.Planar_0:
        return self._elevon1
    
    @property
    def elevon2(self) -> reg.udral.physics.dynamics.rotation.Planar_0:
        return self._elevon2
    
    @property
    def tilt(self) -> reg.udral.physics.dynamics.rotation.Planar_0:
        return self._tilt
    
    @property
    def stow(self) -> reg.udral.physics.dynamics.rotation.Planar_0:
        return self._stow
    
    @property
    def esc1(self) -> reg.udral.physics.dynamics.rotation.Planar_0:
        return self._esc1
    
    @property
    def esc2(self) -> reg.udral.physics.dynamics.rotation.Planar_0:
        return self._esc2
    
    @property
    def esc3(self) -> reg.udral.physics.dynamics.rotation.Planar_0:
        return self._esc3
    
    @property
    def esc4(self) -> reg.udral.physics.dynamics.rotation.Planar_0:
        return self._esc4

    @servo_readiness.setter
    def servo_readiness(self, value: int) -> None:
        assert value in [
            reg.udral.service.common.Readiness_0.ENGAGED,
            reg.udral.service.common.Readiness_0.STANDBY,
            reg.udral.service.common.Readiness_0.SLEEP
        ]
        self._servo_readiness = reg.udral.service.common.Readiness_0(value)
    
    @esc_readiness.setter
    def esc_readiness(self, value: int) -> None:
        assert value in [
            reg.udral.service.common.Readiness_0.ENGAGED,
            reg.udral.service.common.Readiness_0.STANDBY,
            reg.udral.service.common.Readiness_0.SLEEP
        ]
        self._esc_readiness = reg.udral.service.common.Readiness_0(value)

    @elevon1.setter
    def elevon1(self, value: float) -> None:
        self._elevon1 = reg.udral.physics.dynamics.rotation.Planar_0(
            reg.udral.physics.kinematics.rotation.Planar_0(
                uavcan.si.unit.angle.Scalar_1(value)
            )
        )

    @elevon2.setter
    def elevon2(self, value: float) -> None:
        self._elevon2 = reg.udral.physics.dynamics.rotation.Planar_0(
            reg.udral.physics.kinematics.rotation.Planar_0(
                uavcan.si.unit.angle.Scalar_1(value)
            )
        )

    @tilt.setter
    def tilt(self, value: float) -> None:
        self._tilt = reg.udral.physics.dynamics.rotation.Planar_0(
            reg.udral.physics.kinematics.rotation.Planar_0(
                uavcan.si.unit.angle.Scalar_1(value)
            )
        )

    @stow.setter
    def stow(self, value: float) -> None:
        self._stow = reg.udral.physics.dynamics.rotation.Planar_0(
            reg.udral.physics.kinematics.rotation.Planar_0(
                uavcan.si.unit.angle.Scalar_1(value)
            )
        )

    @esc1.setter
    def esc1(self, value: float) -> None:
        """Set ESC setpoint in RPM."""
        self._esc1 = reg.udral.physics.dynamics.rotation.Planar_0(
            reg.udral.physics.kinematics.rotation.Planar_0(
                angular_velocity=uavcan.si.unit.angular_velocity.Scalar_1(
                    value * RPM_TO_RADS
                )
            )
        )

    @esc2.setter
    def esc2(self, value: float) -> None:
        """Set ESC setpoint in RPM."""
        self._esc2 = reg.udral.physics.dynamics.rotation.Planar_0(
            reg.udral.physics.kinematics.rotation.Planar_0(
                angular_velocity=uavcan.si.unit.angular_velocity.Scalar_1(
                    value * RPM_TO_RADS
                )
            )
        )

    @esc3.setter
    def esc3(self, value: float) -> None:
        """Set ESC setpoint in RPM."""
        self._esc3 = reg.udral.physics.dynamics.rotation.Planar_0(
            reg.udral.physics.kinematics.rotation.Planar_0(
                angular_velocity=uavcan.si.unit.angular_velocity.Scalar_1(
                    value * RPM_TO_RADS
                )
            )
        )

    @esc4.setter
    def esc4(self, value: float) -> None:
        """Set ESC setpoint in RPM."""
        self._esc4 = reg.udral.physics.dynamics.rotation.Planar_0(
            reg.udral.physics.kinematics.rotation.Planar_0(
                angular_velocity=uavcan.si.unit.angular_velocity.Scalar_1(
                    value * RPM_TO_RADS
                )
            )
        )


class Waypoint:
    """Stores information for a waypoint."""
    count = 0
    def __init__(self, latitude: float, longitude: float, altitude: float = None, name: str | None = None) -> None:
        self.latitude = latitude
        self.longitude = longitude
        self.altitude = altitude

        Waypoint.count += 1
        self.name = name if name is not None else f"Waypoint {Waypoint.count}"


class MainIO:
    """Main Input/Output class for UAVCAN communication.

    This class serves as the interface for communication with UAVCAN 
    devices and nodes. It initializes and runs a UAVCAN node, subscribes
    to specific topics, and publishes control data.

    Parameters
    ----------
    main : 'Main'
        Reference to the main class.
    freq : int, optional
        Frequency setting for the UAVCAN node, by default DEFAULT_FREQ.

    Attributes
    ----------
    main : 'Main'
        Reference to the main class.

    Methods
    -------
    boot()
        Perform boot-related tasks.
    run()
        Execute control logic and handle UAVCAN communication.
    close()
        Close the MainIO node and release resources.
    """

    class NodeManager:
        """Find node ids corresponding with critical components."""
        class Node:
            defaults = (0, pycyphal.application.node_tracker.Entry(uavcan.node.Heartbeat_1(), None).heartbeat)
            def __init__(self) -> None:
                self.id: int
                self.heartbeat: uavcan.node.Heartbeat_1
                self.id, self.heartbeat = MainIO.NodeManager.Node.defaults
        
        def __init__(self, node: pycyphal.application.Node, config: ConfigParser | None = None) -> None:
            # If a config is passed, fixed node ids will be used.
            self.clock = MainIO.NodeManager.Node()
            self.sensorhub = MainIO.NodeManager.Node()
            self.motorhub = MainIO.NodeManager.Node()
            self.gps = MainIO.NodeManager.Node()
            
            if config:
                self.clock.id = config.getint('node_ids', 'clock')
                self.sensorhub.id = config.getint('node_ids', 'sensorhub')
                self.motorhub.id = config.getint('node_ids', 'motorhub')
                self.gps.id = config.getint('node_ids', 'gps')
            
            self._node = node

            self._sub_heartbeat = self._node.make_subscriber(uavcan.node.Heartbeat_1)

            self.cln_clock_exec_cmd = self._node.make_client(uavcan.node.ExecuteCommand_1, self.clock.id)
            self.cln_sensorhub_exec_cmd = self._node.make_client(uavcan.node.ExecuteCommand_1, self.sensorhub.id)
            self.cln_motorhub_exec_cmd = self._node.make_client(uavcan.node.ExecuteCommand_1, self.motorhub.id)
            self.cln_gps_exec_cmd = self._node.make_client(uavcan.node.ExecuteCommand_1, self.gps.id)

            def update_heartbeat(msg: uavcan.node.Heartbeat_1, info: pycyphal.transport.TransferFrom) -> None:
                if info.source_node_id == self.clock.id:
                    self.clock.heartbeat = msg
                elif info.source_node_id == self.sensorhub.id:
                    self.sensorhub.heartbeat = msg
                elif info.source_node_id == self.motorhub.id:
                    self.motorhub.heartbeat = msg
                elif info.source_node_id == self.gps.id:
                    self.gps.heartbeat = msg
            self._sub_heartbeat.receive_in_background(update_heartbeat)

        def update(self, id: int, old: pycyphal.application.node_tracker.Entry | None, new: pycyphal.application.node_tracker.Entry | None) -> None:
            if new is None: # node offline
                self._destroy_values(id)
            elif new.info is not None: # node has info
                self._update_values(id, new)

        def _update_values(self, id: int, entry: pycyphal.application.node_tracker.Entry) -> None:
            name = ''.join(chr(c) for c in entry.info.name)
            if id==self.clock.id or name=='fmuas.clock':
                self.clock.id = id
                self.cln_clock_exec_cmd.close()
                self.cln_clock_exec_cmd = self._node.make_client(uavcan.node.ExecuteCommand_1, self.clock.id)
                self.clock.heartbeat = entry.heartbeat
            elif id==self.sensorhub.id or name=='fmuas.sensorhub':
                self.sensorhub.id = id
                self.cln_sensorhub_exec_cmd.close()
                self.cln_sensorhub_exec_cmd = self._node.make_client(uavcan.node.ExecuteCommand_1, self.sensorhub.id)
                self.sensorhub.heartbeat = entry.heartbeat
            elif id==self.motorhub.id or name=='fmuas.motorhub':
                self.motorhub.id = id
                self.cln_motorhub_exec_cmd.close()
                self.cln_motorhub_exec_cmd = self._node.make_client(uavcan.node.ExecuteCommand_1, self.motorhub.id)
                self.motorhub.heartbeat = entry.heartbeat
            elif id==self.gps.id or name=='fmuas.gps':
                self.gps.id = id
                self.cln_gps_exec_cmd.close()
                self.cln_gps_exec_cmd = self._node.make_client(uavcan.node.ExecuteCommand_1, self.gps.id)
                self.gps.heartbeat = entry.heartbeat
        
        def _destroy_values(self, id: int) -> None:
            if id==self.clock.id:
                self.clock.id, self.clock.heartbeat = MainIO.NodeManager.Node.defaults
                self.cln_clock_exec_cmd.close()
                self.cln_clock_exec_cmd = self._node.make_client(uavcan.node.ExecuteCommand_1, self.clock.id)
            elif id==self.sensorhub.id:
                self.sensorhub.id, self.sensorhub.heartbeat = MainIO.NodeManager.Node.defaults
                self.cln_sensorhub_exec_cmd.close()
                self.cln_sensorhub_exec_cmd = self._node.make_client(uavcan.node.ExecuteCommand_1, self.sensorhub.id)
            elif id==self.motorhub.id:
                self.motorhub.id, self.motorhub.heartbeat = MainIO.NodeManager.Node.defaults
                self.cln_motorhub_exec_cmd.close()
                self.cln_motorhub_exec_cmd = self._node.make_client(uavcan.node.ExecuteCommand_1, self.motorhub.id)
            elif id==self.gps.id:
                self.gps.id, self.gps.heartbeat = MainIO.NodeManager.Node.defaults
                self.cln_gps_exec_cmd.close()
                self.cln_gps_exec_cmd = self._node.make_client(uavcan.node.ExecuteCommand_1, self.gps.id)

    def __init__(self, main: 'Main', freq: int = DEFAULT_FREQ) -> None:
        """Initialize the MainIO class.

        Parameters
        ----------
        main : 'Main'
            Reference to the main class.
        freq : int, optional
            Frequency setting, by default DEFAULT_FREQ.

        Returns
        -------
        None
        """

        self.main = main
        self._freq = freq

        logging.info("Initializing UAVCAN Node...")

        self._registry = pycyphal.application.make_registry(environment_variables={
            'UAVCAN__NODE__ID'                      :self.main.config.get('node_ids', 'mainio'),
            'UAVCAN__UDP__IFACE'                    :self.main.config.get('main', 'udp'),

            'UAVCAN__PUB__SERVO_READINESS__ID'      :self.main.config.get('subject_ids', 'servo_readiness'),
            'UAVCAN__PUB__ESC_READINESS__ID'        :self.main.config.get('subject_ids', 'esc_readiness'),

            'UAVCAN__PUB__ELEVON1_SP__ID'           :self.main.config.get('subject_ids', 'elevon1_sp'),
            'UAVCAN__SUB__ELEVON1_FEEDBACK__ID'     :self.main.config.get('subject_ids', 'elevon1_feedback'),
            'UAVCAN__SUB__ELEVON1_STATUS__ID'       :self.main.config.get('subject_ids', 'elevon1_status'),
            'UAVCAN__SUB__ELEVON1_POWER__ID'        :self.main.config.get('subject_ids', 'elevon1_power'),
            'UAVCAN__SUB__ELEVON1_DYNAMICS__ID'     :self.main.config.get('subject_ids', 'elevon1_dynamics'),

            'UAVCAN__PUB__ELEVON2_SP__ID'           :self.main.config.get('subject_ids', 'elevon2_sp'),
            'UAVCAN__SUB__ELEVON2_FEEDBACK__ID'     :self.main.config.get('subject_ids', 'elevon2_feedback'),
            'UAVCAN__SUB__ELEVON2_STATUS__ID'       :self.main.config.get('subject_ids', 'elevon2_status'),
            'UAVCAN__SUB__ELEVON2_POWER__ID'        :self.main.config.get('subject_ids', 'elevon2_power'),
            'UAVCAN__SUB__ELEVON2_DYNAMICS__ID'     :self.main.config.get('subject_ids', 'elevon2_dynamics'),

            'UAVCAN__PUB__TILT_SP__ID'              :self.main.config.get('subject_ids', 'tilt_sp'),
            'UAVCAN__SUB__TILT_FEEDBACK__ID'        :self.main.config.get('subject_ids', 'tilt_feedback'),
            'UAVCAN__SUB__TILT_STATUS__ID'          :self.main.config.get('subject_ids', 'tilt_status'),
            'UAVCAN__SUB__TILT_POWER__ID'           :self.main.config.get('subject_ids', 'tilt_power'),
            'UAVCAN__SUB__TILT_DYNAMICS__ID'        :self.main.config.get('subject_ids', 'tilt_dynamics'),

            'UAVCAN__PUB__STOW_SP__ID'              :self.main.config.get('subject_ids', 'stow_sp'),
            'UAVCAN__SUB__STOW_FEEDBACK__ID'        :self.main.config.get('subject_ids', 'stow_feedback'),
            'UAVCAN__SUB__STOW_STATUS__ID'          :self.main.config.get('subject_ids', 'stow_status'),
            'UAVCAN__SUB__STOW_POWER__ID'           :self.main.config.get('subject_ids', 'stow_power'),
            'UAVCAN__SUB__STOW_DYNAMICS__ID'        :self.main.config.get('subject_ids', 'stow_dynamics'),

            'UAVCAN__PUB__ESC1_SP__ID'              :self.main.config.get('subject_ids', 'esc1_sp'),
            'UAVCAN__SUB__ESC1_FEEDBACK__ID'        :self.main.config.get('subject_ids', 'esc1_feedback'),
            'UAVCAN__SUB__ESC1_STATUS__ID'          :self.main.config.get('subject_ids', 'esc1_status'),
            'UAVCAN__SUB__ESC1_POWER__ID'           :self.main.config.get('subject_ids', 'esc1_power'),
            'UAVCAN__SUB__ESC1_DYNAMICS__ID'        :self.main.config.get('subject_ids', 'esc1_dynamics'),

            'UAVCAN__PUB__ESC2_SP__ID'              :self.main.config.get('subject_ids', 'esc2_sp'),
            'UAVCAN__SUB__ESC2_FEEDBACK__ID'        :self.main.config.get('subject_ids', 'esc2_feedback'),
            'UAVCAN__SUB__ESC2_STATUS__ID'          :self.main.config.get('subject_ids', 'esc2_status'),
            'UAVCAN__SUB__ESC2_POWER__ID'           :self.main.config.get('subject_ids', 'esc2_power'),
            'UAVCAN__SUB__ESC2_DYNAMICS__ID'        :self.main.config.get('subject_ids', 'esc2_dynamics'),

            'UAVCAN__PUB__ESC3_SP__ID'              :self.main.config.get('subject_ids', 'esc3_sp'),
            'UAVCAN__SUB__ESC3_FEEDBACK__ID'        :self.main.config.get('subject_ids', 'esc3_feedback'),
            'UAVCAN__SUB__ESC3_STATUS__ID'          :self.main.config.get('subject_ids', 'esc3_status'),
            'UAVCAN__SUB__ESC3_POWER__ID'           :self.main.config.get('subject_ids', 'esc3_power'),
            'UAVCAN__SUB__ESC3_DYNAMICS__ID'        :self.main.config.get('subject_ids', 'esc3_dynamics'),

            'UAVCAN__PUB__ESC4_SP__ID'              :self.main.config.get('subject_ids', 'esc4_sp'),
            'UAVCAN__SUB__ESC4_FEEDBACK__ID'        :self.main.config.get('subject_ids', 'esc4_feedback'),
            'UAVCAN__SUB__ESC4_STATUS__ID'          :self.main.config.get('subject_ids', 'esc4_status'),
            'UAVCAN__SUB__ESC4_POWER__ID'           :self.main.config.get('subject_ids', 'esc4_power'),
            'UAVCAN__SUB__ESC4_DYNAMICS__ID'        :self.main.config.get('subject_ids', 'esc4_dynamics'),

            'UAVCAN__SUB__INERTIAL__ID'             :self.main.config.get('subject_ids', 'inertial'),
            'UAVCAN__SUB__ALTITUDE__ID'             :self.main.config.get('subject_ids', 'altitude'),
            'UAVCAN__SUB__IAS__ID'                  :self.main.config.get('subject_ids', 'ias'),
            'UAVCAN__SUB__AOA__ID'                  :self.main.config.get('subject_ids', 'aoa'),
            'UAVCAN__SUB__GPS__ID'                  :self.main.config.get('subject_ids', 'gps'),

            'UAVCAN__SUB__CLOCK_SYNC_TIME__ID'      :self.main.config.get('subject_ids', 'clock_sync_time'),
            'UAVCAN__SUB__GPS_SYNC_TIME__ID'        :self.main.config.get('subject_ids', 'gps_sync_time'),
        })

        self._use_gps_time = False
        
        node_info = uavcan.node.GetInfo_1.Response(
            software_version=uavcan.node.Version_1(major=1, minor=0),
            name=f'fmuas.uasmain{self.main.systemid}.mainio',
        )

        self._node = pycyphal.application.make_node(node_info, self._registry)

        self.node_manager = MainIO.NodeManager(self._node, config=self.main.config) # remove config for nonfixed node ids
        self._tracker = pycyphal.application.node_tracker.NodeTracker(self._node)
        self._tracker.add_update_handler(self.node_manager.update)

        self._node.heartbeat_publisher.mode = uavcan.node.Mode_1.INITIALIZATION
        self._node.heartbeat_publisher.vendor_specific_status_code = os.getpid() % 100
        
        self._sub_ins = self._node.make_subscriber(reg.udral.physics.kinematics.cartesian.StateVarTs_0, 'inertial')
        self._sub_alt = self._node.make_subscriber(uavcan.si.unit.length.WideScalar_1, 'altitude')
        self._sub_ias = self._node.make_subscriber(reg.udral.physics.kinematics.translation.LinearTs_0, 'ias')
        self._sub_aoa = self._node.make_subscriber(uavcan.si.unit.angle.Scalar_1, 'aoa')
        self._sub_gps = self._node.make_subscriber(reg.udral.physics.kinematics.geodetic.PointStateVarTs_0, 'gps')

        self._pub_servo_readiness = self._node.make_publisher(reg.udral.service.common.Readiness_0, 'servo_readiness')
        self._pub_esc_readiness = self._node.make_publisher(reg.udral.service.common.Readiness_0, 'esc_readiness')

        self._pub_elevon1_sp = self._node.make_publisher(reg.udral.physics.dynamics.rotation.Planar_0, 'elevon1_sp')
        self._sub_elevon1_feedback = self._node.make_subscriber(reg.udral.service.actuator.common.Feedback_0, 'elevon1_feedback')
        self._sub_elevon1_status = self._node.make_subscriber(reg.udral.service.actuator.common.Status_0, 'elevon1_status')
        self._sub_elevon1_power = self._node.make_subscriber(reg.udral.physics.electricity.PowerTs_0, 'elevon1_power')
        self._sub_elevon1_dynamics = self._node.make_subscriber(reg.udral.physics.dynamics.rotation.PlanarTs_0, 'elevon1_dynamics')

        self._pub_elevon2_sp = self._node.make_publisher(reg.udral.physics.dynamics.rotation.Planar_0, 'elevon2_sp')
        self._sub_elevon2_feedback = self._node.make_subscriber(reg.udral.service.actuator.common.Feedback_0, 'elevon2_feedback')
        self._sub_elevon2_status = self._node.make_subscriber(reg.udral.service.actuator.common.Status_0, 'elevon2_status')
        self._sub_elevon2_power = self._node.make_subscriber(reg.udral.physics.electricity.PowerTs_0, 'elevon2_power')
        self._sub_elevon2_dynamics = self._node.make_subscriber(reg.udral.physics.dynamics.rotation.PlanarTs_0, 'elevon2_dynamics')

        self._pub_tilt_sp = self._node.make_publisher(reg.udral.physics.dynamics.rotation.Planar_0, 'tilt_sp')
        self._sub_tilt_feedback = self._node.make_subscriber(reg.udral.service.actuator.common.Feedback_0, 'tilt_feedback')
        self._sub_tilt_status = self._node.make_subscriber(reg.udral.service.actuator.common.Status_0, 'tilt_status')
        self._sub_tilt_power = self._node.make_subscriber(reg.udral.physics.electricity.PowerTs_0, 'tilt_power')
        self._sub_tilt_dynamics = self._node.make_subscriber(reg.udral.physics.dynamics.rotation.PlanarTs_0, 'tilt_dynamics')

        self._pub_stow_sp = self._node.make_publisher(reg.udral.physics.dynamics.rotation.Planar_0, 'stow_sp')
        self._sub_stow_feedback = self._node.make_subscriber(reg.udral.service.actuator.common.Feedback_0, 'stow_feedback')
        self._sub_stow_status = self._node.make_subscriber(reg.udral.service.actuator.common.Status_0, 'stow_status')
        self._sub_stow_power = self._node.make_subscriber(reg.udral.physics.electricity.PowerTs_0, 'stow_power')
        self._sub_stow_dynamics = self._node.make_subscriber(reg.udral.physics.dynamics.rotation.PlanarTs_0, 'stow_dynamics')

        self._pub_esc1_sp = self._node.make_publisher(reg.udral.physics.dynamics.rotation.Planar_0, 'esc1_sp')
        self._sub_esc1_feedback = self._node.make_subscriber(reg.udral.service.actuator.common.Feedback_0, 'esc1_feedback')
        self._sub_esc1_status = self._node.make_subscriber(reg.udral.service.actuator.common.Status_0, 'esc1_status')
        self._sub_esc1_power = self._node.make_subscriber(reg.udral.physics.electricity.PowerTs_0, 'esc1_power')
        self._sub_esc1_dynamics = self._node.make_subscriber(reg.udral.physics.dynamics.rotation.PlanarTs_0, 'esc1_dynamics')

        self._pub_esc2_sp = self._node.make_publisher(reg.udral.physics.dynamics.rotation.Planar_0, 'esc2_sp')
        self._sub_esc2_feedback = self._node.make_subscriber(reg.udral.service.actuator.common.Feedback_0, 'esc2_feedback')
        self._sub_esc2_status = self._node.make_subscriber(reg.udral.service.actuator.common.Status_0, 'esc2_status')
        self._sub_esc2_power = self._node.make_subscriber(reg.udral.physics.electricity.PowerTs_0, 'esc2_power')
        self._sub_esc2_dynamics = self._node.make_subscriber(reg.udral.physics.dynamics.rotation.PlanarTs_0, 'esc2_dynamics')

        self._pub_esc3_sp = self._node.make_publisher(reg.udral.physics.dynamics.rotation.Planar_0, 'esc3_sp')
        self._sub_esc3_feedback = self._node.make_subscriber(reg.udral.service.actuator.common.Feedback_0, 'esc3_feedback')
        self._sub_esc3_status = self._node.make_subscriber(reg.udral.service.actuator.common.Status_0, 'esc3_status')
        self._sub_esc3_power = self._node.make_subscriber(reg.udral.physics.electricity.PowerTs_0, 'esc3_power')
        self._sub_esc3_dynamics = self._node.make_subscriber(reg.udral.physics.dynamics.rotation.PlanarTs_0, 'esc3_dynamics')

        self._pub_esc4_sp = self._node.make_publisher(reg.udral.physics.dynamics.rotation.Planar_0, 'esc4_sp')
        self._sub_esc4_feedback = self._node.make_subscriber(reg.udral.service.actuator.common.Feedback_0, 'esc4_feedback')
        self._sub_esc4_status = self._node.make_subscriber(reg.udral.service.actuator.common.Status_0, 'esc4_status')
        self._sub_esc4_power = self._node.make_subscriber(reg.udral.physics.electricity.PowerTs_0, 'esc4_power')
        self._sub_esc4_dynamics = self._node.make_subscriber(reg.udral.physics.dynamics.rotation.PlanarTs_0, 'esc4_dynamics')

        self._sub_clock_sync_time = self._node.make_subscriber(uavcan.time.SynchronizedTimestamp_1, 'clock_sync_time')
        self._sub_clock_sync_time_last = self._node.make_subscriber(uavcan.time.Synchronization_1)

        self._sub_gps_sync_time = self._node.make_subscriber(uavcan.time.SynchronizedTimestamp_1, 'gps_sync_time')
        self._sub_gps_sync_time_last = self._node.make_subscriber(uavcan.time.Synchronization_1)

        self._cln_clock_sync_info = self._node.make_client(uavcan.time.GetSynchronizationMasterInfo_0, self.main.config.getint('node_ids', 'clock'))
        self._cln_gps_sync_info = self._node.make_client(uavcan.time.GetSynchronizationMasterInfo_0, self.main.config.getint('node_ids', 'gps'))

        self._srv_exec_cmd = self._node.get_server(uavcan.node.ExecuteCommand_1)
        self._srv_exec_cmd.serve_in_background(self._serve_exec_cmd)

        self._node.start()

        logging.info("MainIO initialized")

    async def _serve_exec_cmd(
            self,
            request: uavcan.node.ExecuteCommand_1.Request, 
            metadata: pycyphal.presentation.ServiceRequestMetadata,) -> uavcan.node.ExecuteCommand_1.Response:
        logging.info("Execute command request %s from node %d", request, metadata.client_node_id)
        match request.command:
            case uavcan.node.ExecuteCommand_1.Request.COMMAND_POWER_OFF:
                return uavcan.node.ExecuteCommand_1.Response(
                    uavcan.node.ExecuteCommand_1.Response.STATUS_NOT_AUTHORIZED
                )
            case uavcan.node.ExecuteCommand_1.Request.COMMAND_RESTART:
                return uavcan.node.ExecuteCommand_1.Response(
                    uavcan.node.ExecuteCommand_1.Response.STATUS_NOT_AUTHORIZED
                )
            case uavcan.node.ExecuteCommand_1.Request.COMMAND_EMERGENCY_STOP:
                return uavcan.node.ExecuteCommand_1.Response(
                    uavcan.node.ExecuteCommand_1.Response.STATUS_NOT_AUTHORIZED
                )
            case uavcan.node.ExecuteCommand_1.Request.COMMAND_FACTORY_RESET:
                return uavcan.node.ExecuteCommand_1.Response(
                    uavcan.node.ExecuteCommand_1.Response.STATUS_NOT_AUTHORIZED
                )
            case uavcan.node.ExecuteCommand_1.Request.COMMAND_STORE_PERSISTENT_STATES:
                return uavcan.node.ExecuteCommand_1.Response(
                    uavcan.node.ExecuteCommand_1.Response.STATUS_NOT_AUTHORIZED
                )
            case uavcan.node.ExecuteCommand_1.Request.COMMAND_BEGIN_SOFTWARE_UPDATE:
                return uavcan.node.ExecuteCommand_1.Response(
                    uavcan.node.ExecuteCommand_1.Response.STATUS_NOT_AUTHORIZED
                )
            case _:
                return uavcan.node.ExecuteCommand_1.Response(
                    uavcan.node.ExecuteCommand_1.Response.STATUS_BAD_COMMAND
                )

    async def boot(self) -> None:
        """Perform boot-related tasks."""
        # TODO: do boot stuff here, maybe read a different .ini?
        await asyncio.sleep(0)

    @async_loop_decorator()
    async def _mainio_run_loop(self) -> None:
        await self._pub_servo_readiness.publish(self.main.txdata.servo_readiness)
        await self._pub_esc_readiness.publish(self.main.txdata.esc_readiness)

        await self._pub_elevon1_sp.publish(self.main.txdata.elevon1)
        await self._pub_elevon2_sp.publish(self.main.txdata.elevon2)
        await self._pub_tilt_sp.publish(self.main.txdata.tilt)
        await self._pub_stow_sp.publish(self.main.txdata.stow)

        await self._pub_esc1_sp.publish(self.main.txdata.esc1)
        await self._pub_esc2_sp.publish(self.main.txdata.esc2)
        await self._pub_esc3_sp.publish(self.main.txdata.esc3)
        await self._pub_esc4_sp.publish(self.main.txdata.esc4)

        await asyncio.sleep(1 / self._freq)

    async def run(self) -> None:
        """Recieve subscripted data and write to publishers."""
        #region Subscriptions
        def on_time(msg: uavcan.time.SynchronizedTimestamp_1, _: pycyphal.transport.TransferFrom) -> None:
            self.main.rxdata.time.dump(msg)
        self._sub_clock_sync_time.receive_in_background(on_time)

        # TODO: gps time transition
        def on_gps_time(msg: uavcan.time.SynchronizedTimestamp_1, _: pycyphal.transport.TransferFrom) -> None:
            if self._use_gps_time:
                self.main.rxdata.time.dump(msg)
        self._sub_gps_sync_time.receive_in_background(on_gps_time)
        
        def on_att(msg: reg.udral.physics.kinematics.cartesian.StateVarTs_0, _: pycyphal.transport.TransferFrom) -> None:
            self.main.rxdata.att.dump(msg)
        self._sub_ins.receive_in_background(on_att)

        def on_alt(msg: uavcan.si.unit.length.WideScalar_1, _: pycyphal.transport.TransferFrom) -> None:
            t = self.main.rxdata.time.time
            self.main.rxdata.alt.dump(msg, t)
        self._sub_alt.receive_in_background(on_alt)

        def on_gps(msg: reg.udral.physics.kinematics.geodetic.PointStateVarTs_0, _: pycyphal.transport.TransferFrom) -> None:
            self.main.rxdata.gps.dump(msg, self.main.rxdata.att.yaw)
        self._sub_gps.receive_in_background(on_gps)

        def on_ias(msg: reg.udral.physics.kinematics.translation.LinearTs_0, _: pycyphal.transport.TransferFrom) -> None:
            self.main.rxdata.ias.dump(msg)
        self._sub_ias.receive_in_background(on_ias)

        def on_aoa(msg: uavcan.si.unit.angle.Scalar_1, _: pycyphal.transport.TransferFrom) -> None:
            t = self.main.rxdata.time.time
            self.main.rxdata.aoa.dump(msg, t)
        self._sub_aoa.receive_in_background(on_aoa)
        
        # TODO: motor diagnostic subscribers
        #endregion
        
        self._node.heartbeat_publisher.mode = uavcan.node.Mode_1.OPERATIONAL
        logging.warning("UAVCAN Node Running...\n----- Ctrl-C to exit -----") # TODO: no verification

        await self._mainio_run_loop()

    def close(self) -> None:
        """Close the instance."""
        logging.info("Closing MainIO") # TODO: Change to logging.debug()
        self._node.close()


class Navigator:
    """Converts navigational commands to autopilot commands."""
    def __init__(self, main: 'Main') -> None:
        """Initializes the Navigator class.

        Parameters
        ----------
        main : 'Main'
            The main object.
        """

        self.main = main
        self._waypoint_list: list[Waypoint] = [] # Navigating from waypoint 0 to waypoint 1
        self._current_waypoint = None
        self.commanded_heading = 0.0 # DEGREES
        self.commanded_altitude = self.main.processor.spf_altitude

    async def boot(self) -> None:
        """Perform boot-related tasks."""
        # TODO: do boot stuff here, calibrate gps etc
        # TODO: await gps online
        self.append_wpt(Waypoint(self.main.rxdata.gps.latitude, self.main.rxdata.gps.longitude, self.main.rxdata.gps.altitude, name='Start'))
        self._current_waypoint = self._waypoint_list[0]
        logging.debug(f"Navigator aligning at {self._waypoint_list[0].latitude}, {self._waypoint_list[0].longitude}")
        # TODO: load flight plan from a file
        await asyncio.sleep(0)

    def append_wpt(self, *wpts: Waypoint) -> None:
        """Add a Waypoint to the end of the flight plan."""
        for wpt in wpts:
            self._waypoint_list.append(wpt)

    def next_wpt(self, wpt: Waypoint) -> None:
        """Direct to a waypoint."""
        self._waypoint_list.insert(1, wpt)

    def calc_heading(self) -> float:
        """"Determine the desired heading to the next waypoint."""
        if len(self._waypoint_list)>1:
            lata = self.main.rxdata.gps.latitude
            latb = self._waypoint_list[1].latitude
            dlon = self._waypoint_list[1].longitude - self.main.rxdata.gps.longitude

            x = math.cos(latb) * math.sin(dlon)
            y = math.cos(lata)*math.sin(latb) - math.sin(lata)*math.cos(latb)*math.cos(dlon)
            
            self.commanded_heading = -math.degrees(math.atan2(x, y))
            self.commanded_heading %= 360.0
        else:
            self.commanded_heading = self.main.rxdata.att.yaw + 15.0 # Enter right hand continuous turn
            self.commanded_heading %= 360.0
        return self.commanded_heading # DEGREES
    
    def calc_altitude(self) -> float:
        """Determine the desired altitude from the next waypoint."""
        if len(self._waypoint_list)>1:
            self.commanded_altitude = self._waypoint_list[1].altitude if self._waypoint_list[1].altitude is not None else self.main.processor.spf_altitude
        else:
            # TODO
            pass
        return self.commanded_altitude

    @async_loop_decorator(close=False)
    async def _navigator_run_loop(self) -> None:
        """Set processor setpoints based on flight plan."""
        # TODO: navigator modes, safety checks
        self.main.processor.spf_heading = math.radians(self.calc_heading())
        self.main.processor.spf_altitude = self.calc_altitude()
        await asyncio.sleep(0)

    async def run(self) -> None:
        """Calculate desired heading from flight plan."""
        logging.info("Starting Navigator")
        await self._navigator_run_loop()


class Processor:
    """Manages various calculations and controls for the system.

    This class handles the calculation of servo commands and throttle 
    outputs based on sensor data and control setpoints.

    Attributes
    ----------
    main : 'Main'
        The main object representing the core of the system.
    spf_altitude : float
        Setpoint for altitude in meters.
    spf_heading : float
        Setpoint for heading in radians.
    spf_ias : float
        Setpoint for indicated airspeed.

    Methods
    -------
    boot(self)
        Initialize and perform boot-related tasks.
    _flight_servos(self)
        Calculate flight servo commands.
    _vtol_servos(self)
        Calculate VTOL servo commands.
    _flight_throttles(self)
        Calculate flight throttle outputs.
    _vtol_throttles(self)
        Calculate VTOL throttle outputs.
    run(self)
        Run the Processor and execute control logic.
    """

    MAX_THROTTLE = 14000

    def __init__(self, main: 'Main') -> None:
        """Initialize the Processor class.

        Parameters
        ----------
        main : 'Main'
            The main object.
        """

        self.main = main

        self._vpath = 0.0
        self._dyaw = 0.0
        self._ias_scalar = 1.0

        self._servos: np.ndarray = np.zeros(4, dtype=np.float16)
        self._throttles: np.ndarray = np.zeros(4, dtype=np.float16)

        self._fservos = np.zeros(4, dtype=np.float16) # elevons*2, rudder, wingtilt
        self._vservos = np.zeros(4, dtype=np.float16)

        self._fthrottles = np.zeros(4, dtype=np.float16) # throttles*4
        self._vthrottles = np.zeros(4, dtype=np.float16)

        self.spf_altitude = 100.0
        self.spf_heading = 0.0
        self.spf_ias = 40.0

        self._spf_vpath = 0.0
        self._spf_aoa = 0.1
        self._spf_roll = 0.0
        self._spf_rollspeed = 0.0

        self.spv_altitude = 100.0
        self.spv_heading = 0.0

        self._spv_xspeed = 0.0
        self._spv_yspeed = 0.0
        self._spv_pitchspeed = 0.0
        self._spv_rollspeed = 0.0
        self._spv_pitch = 0.0
        self._spv_roll = 0.0
        self._spv_yawspeed = 0.0
        self._spv_vs = 0.0

        self._outf_pitch = 0.0
        self._outf_roll = 0.0
        self._outf_throttle_ias = 0.0
        self._throttle_roll_corr = 0.0
        self._throttle_vpa_corr = 0.0

        self._outv_pitch = 0.0
        self._outv_roll = 0.0
        self._outv_yaw = 0.0
        self._outv_throttle = 0.0

        # self._pid{f or v}_{from}_{to}

        self._pidf_alt_vpa = PID(kp=0.007, ti=0.01, td=0.1, integral_limit=0.05, maximum=0.05, minimum=-0.05)
        self._pidf_vpa_aoa = PID(kp=0.7, ti=3.0, td=0.05, integral_limit=0.2, maximum=math.pi/8, minimum=-0.05)
        self._pidf_aoa_out = PID(kp=-0.07, ti=-0.008, td=0.02, integral_limit=1, maximum=0.0, minimum=-math.pi/12)
        self._pidf_dyw_rol = PID(kp=-0.75, ti=-8.0, td=0.002, integral_limit=0.1, maximum=math.pi/6, minimum=-math.pi/6)
        self._pidf_rol_rls = PID(kp=1.5, ti=6.0, td=0.02, integral_limit=0.2, maximum=2.0, minimum=-2.0)
        self._pidf_rls_out = PID(kp=0.005, ti=0.003, td=0.005, integral_limit=0.1, maximum=0.1, minimum=-0.1)
        self._pidf_ias_thr = PID(kp=0.1, ti=0.0, td=0.0, integral_limit=1.0, maximum=1.00, minimum=0.02) # TODO

        self._pidv_xdp_xsp = PID(kp=0.0, ti=0.0, td=0.0, integral_limit=None, minimum=-5.0, maximum=5.0)
        self._pidv_xsp_rol = PID(kp=0.0, ti=0.0, td=0.0, integral_limit=None, minimum=-math.pi/12, maximum=math.pi/12)
        self._pidv_rol_rls = PID(kp=1.5, ti=0.5, td=0.0, integral_limit=None, minimum=-math.pi/6, maximum=math.pi/6)
        # self._pidv_rls_out = PID(kp=0.02, ti=0.15, td=0.08, integral_limit=None, minimum=-0.08, maximum=0.08)
        self._pidv_rls_out = PID(kp=0.018, ti=0.1, td=0.05, integral_limit=0.2, minimum=-0.08, maximum=0.08)

        self._pidv_ydp_ysp = PID(kp=0.0, ti=0.0, td=0.0, integral_limit=None, minimum=-5.0, maximum=5.0)
        self._pidv_ysp_pit = PID(kp=0.0, ti=0.0, td=0.0, integral_limit=None, minimum=-math.pi/12, maximum=math.pi/12)
        self._pidv_pit_pts = PID(kp=1.0, ti=0.5, td=0.0, integral_limit=None, minimum=-math.pi/6, maximum=math.pi/6)
        # self._pidv_pts_out = PID(kp=0.033, ti=0.12, td=0.08, integral_limit=None, minimum=-0.1, maximum=0.1)
        self._pidv_pts_out = PID(kp=0.025, ti=0.04, td=0.065, integral_limit=1.0, minimum=-0.1, maximum=0.1)

        self._pidv_alt_vsp = PID(kp=0.0, ti=0.0, td=0.0, integral_limit=None, minimum=-50.0, maximum=100.0)
        self._pidv_vsp_out = PID(kp=0.0, ti=0.0, td=0.0, integral_limit=None, minimum=0.0, maximum=1.0)

        self._pidv_dyw_yws = PID(kp=0.0, ti=0.0, td=0.0, integral_limit=None, minimum=-math.pi/6, maximum=-math.pi/6)
        self._pidv_yws_out = PID(kp=0.0, ti=0.0, td=0.0, integral_limit=None, minimum=-0.3, maximum=0.3)

    async def boot(self) -> None:
        """Perform boot-related tasks."""
        # TODO: do boot stuff here, maybe read a different .ini?
        self.main.state.inc_mode()
        await asyncio.sleep(0)

    #region Calculations
    def _flight_calc(self) -> np.ndarray:
        """Calculate flight servo commands from sensors."""
        if (alt:=self.main.rxdata.alt).dt > 0.0:
            self._spf_vpath = self._pidf_alt_vpa.cycle(alt.altitude, self.spf_altitude, alt.dt)
            self.main.rxdata.alt.dt = 0.0

        if any([(att:=self.main.rxdata.att).dt > 0.0, (aoa:=self.main.rxdata.aoa).dt > 0.0]):
            if not aoa.dt > 0.0:
                dt = att.dt
            elif not att.dt > 0.0:
                dt = aoa.dt
            else:
                dt = (att.dt + aoa.dt) / 2

            self._vpath = att.pitch - math.cos(att.roll) * aoa.aoa
            self._spf_aoa = self._pidf_vpa_aoa.cycle(self._vpath, self._spf_vpath, dt) # TODO
            self._throttle_vpa_corr = 4*(self._spf_vpath-self._vpath) if self._vpath<self._spf_vpath-0.05 else 0.0

        if (att:=self.main.rxdata.att).dt > 0.0:
            self._dyaw = calc_dyaw(att.yaw, self.spf_heading)

            self._spf_roll = self._pidf_dyw_rol.cycle(self._dyaw, 0.0, att.dt)
            self._spf_rollspeed = self._pidf_rol_rls.cycle(att.roll, self._spf_roll, att.dt)
            self._outf_roll = self._pidf_rls_out.cycle(att.rollspeed, self._spf_rollspeed, att.dt)

            self._throttle_roll_corr = abs(math.sin(att.roll)) if abs(att.roll)>math.pi/24 else 0.0

            self.main.rxdata.att.dt = 0.0

        if (aoa:=self.main.rxdata.aoa).dt > 0.0:
            self._outf_pitch = self._pidf_aoa_out.cycle(aoa.aoa, self._spf_aoa, aoa.dt)
            self.main.rxdata.aoa.dt = 0.0

        if (ias:=self.main.rxdata.ias).dt > 0.0:
            self._outf_throttle_ias = self._pidf_ias_thr.cycle(ias.ias, self.spf_ias, ias.dt)
            ias.dt = 0.0

        self._fservos[0] = self._outf_pitch + self._outf_roll # TODO
        self._fservos[1] = self._outf_pitch - self._outf_roll # TODO
        self._fservos[2:].fill(0.0)
        self._fservos = self._fservos.clip(-10.0, 10.0) # overflow protection (not servo limits)

        self._throttle_roll_corr = min(self._throttle_roll_corr, 1.0)
        self._throttle_vpa_corr = min(self._throttle_vpa_corr, 1.0)

        self._fthrottles.fill(self._outf_throttle_ias + self._throttle_vpa_corr + self._throttle_roll_corr)
        self._fthrottles = self._fthrottles.clip(0.0, 1.0)
        self._fthrottles *= Processor.MAX_THROTTLE

        return self._fservos, self._fthrottles

    def _vtol_calc(self) -> np.ndarray:
        """Calculate VTOL throttle commands from sensors."""
        if (alt:=self.main.rxdata.alt).dt > 0.0:
            self._spv_vs = self._pidv_alt_vsp.cycle(alt.altitude, self.spv_altitude, alt.dt)
            self._outv_throttle = self._pidv_vsp_out.cycle(alt.vs, self._spv_vs, alt.dt)
            self.main.rxdata.alt.dt = 0.0

        if (cam:=self.main.rxdata.cam).dt > 0.0:
            self._spv_xspeed = self._pidv_xdp_xsp.cycle(cam.xdp, 0.0, cam.dt)
            self._spv_yspeed = self._pidv_ydp_ysp.cycle(cam.ydp, 0.0, cam.dt)
            self.main.rxdata.cam.dt = 0.0

        if (gps:=self.main.rxdata.gps).dt > 0.0:
            self._spv_roll = self._pidv_xsp_rol.cycle(gps.xspeed, self._spv_xspeed, gps.dt)
            self._spv_pitch = self._pidv_ysp_pit.cycle(gps.yspeed, self._spv_yspeed, gps.dt)
            self.main.rxdata.gps.dt = 0.0

        if (att:=self.main.rxdata.att).dt > 0.0:
            self._spv_rollspeed = self._pidv_rol_rls.cycle(att.roll, self._spv_roll, att.dt)
            self._spv_pitchspeed = self._pidv_pit_pts.cycle(att.pitch, self._spv_pitch, att.dt)
            self._outv_roll = self._pidv_rls_out.cycle(att.rollspeed, self._spv_rollspeed, att.dt)
            self._outv_pitch = self._pidv_pts_out.cycle(att.pitchspeed, self._spv_pitchspeed, att.dt)

            self._dyaw = calc_dyaw(att.yaw, self.spf_heading)
            # self._spv_yawspeed = self._pidv_dyw_yws.cycle(self._dyaw, 0.0, att.dt)
            self._outv_yaw = self._pidv_yws_out.cycle(att.yawspeed, self._spv_yawspeed, att.dt)
            self.main.rxdata.att.dt = 0.0

        t1 = self._outv_pitch + self._outv_roll - self._outv_yaw
        t2 = self._outv_pitch - self._outv_roll + self._outv_yaw
        t3 = -self._outv_pitch + self._outv_roll + self._outv_yaw
        t4 = -self._outv_pitch - self._outv_roll - self._outv_yaw
        self._vthrottles = np.array([t1, t2, t3, t4], dtype=np.float16)
        self._vthrottles *= Processor.MAX_THROTTLE

        self._vservos.fill(math.pi/2)
        if self.main.state.custom_submode == g.CUSTOM_SUBMODE_TAKEOFF_HOVER:
            self._vservos[3] = 0.0

        return self._vservos, self._vthrottles
    #endregion

    @async_loop_decorator()
    async def _processor_run_loop(self) -> None:
        try:
            self._ias_scalar = 1296 / (self.main.rxdata.ias.ias**2)
        except ZeroDivisionError:
            self._ias_scalar = 1.0

        self._ias_scalar = min(self._ias_scalar, 10.0)

        # TODO: setpoints
        match self.main.state.custom_submode:
            case g.CUSTOM_SUBMODE_TAKEOFF_ASCENT | g.CUSTOM_SUBMODE_TAKEOFF_HOVER | g.CUSTOM_SUBMODE_LANDING_DESCENT | g.CUSTOM_SUBMODE_LANDING_HOVER:
                # VTOL
                self._servos, self._throttles = self._vtol_calc()
            case g.CUSTOM_SUBMODE_TAKEOFF_TRANSIT | g.CUSTOM_SUBMODE_LANDING_TRANSIT:
                # Transit modes
                # TODO mixing (DO NOT USE BOTH FUNCTIONS BECAUSE RACE CONDITION TO RESET dt)
                pass
            case g.CUSTOM_SUBMODE_FLIGHT_NORMAL | g.CUSTOM_SUBMODE_FLIGHT_TERRAIN_AVOIDANCE:
                # Normal flight
                self._servos, self._throttles = self._flight_calc()
                self._servos *= self._ias_scalar
            case g.CUSTOM_SUBMODE_FLIGHT_MANUAL:
                # Manual flight via GCS
                # TODO switch from lua to GCS
                pass
            case _:
                # Safed
                self._servos = np.array([0.0, 0.0, math.pi/2, math.pi/2], dtype=np.float16)
                self._throttles = np.zeros(4, dtype=np.float16)

        self._servos[:2] = np.clip(self._servos[:2], -math.pi/12, 7*math.pi/12)
        self._servos[2:] = np.clip(self._servos[2:], 0.0, math.pi/2)
        self._throttles = np.clip(self._throttles, -14000, 14000) # TODO: reset min to 0
        self._servos = np.nan_to_num(self._servos)
        self._throttles = np.nan_to_num(self._throttles)

        self.main.txdata.elevon1 = self._servos[0]
        self.main.txdata.elevon2 = self._servos[1]
        self.main.txdata.tilt = self._servos[2]
        self.main.txdata.stow = self._servos[3]
        
        self.main.txdata.esc1 = self._throttles[0]
        self.main.txdata.esc2 = self._throttles[1]
        self.main.txdata.esc3 = self._throttles[2]
        self.main.txdata.esc4 = self._throttles[3]

        await asyncio.sleep(0)

    async def run(self) -> None:
        """Calculate desired control positions."""
        logging.info("Starting Processor")
        await self._processor_run_loop()

    def close(self) -> None:
        logging.info("Closing Processor")
        pass


class ImageProcessor:
    """Find and process new images."""
    def __init__(self, main: 'Main', freq: int = 1) -> None:
        self.main = main
        self._freq = freq
        self._path = os.path.join(os.getcwd(), 'stored_images')

    @async_loop_decorator(close=False)
    async def _image_processor_run_loop(self):
        """Find and process new images."""
        previous_file_list = [f for f in os.listdir(self._path) if os.path.isfile(os.path.join(self._path, f))]

        await asyncio.sleep(1 / self._freq)

        new_file_list = [f for f in os.listdir(self._path) if os.path.isfile(os.path.join(self._path, f))]
        file_diff = [x for x in new_file_list if x not in previous_file_list]
        previous_file_list = new_file_list

        if self.main.state.custom_mode == g.CUSTOM_MODE_LANDING or True: # TODO
            if len(file_diff) != 0:
                for f in file_diff:
                    logging.info(f"Proccesing image {f}")
                    # task = asyncio.create_task()
                    if out := await img.find_h(os.path.join(self._path, f), display=False):
                        dx, dy, confidence, image = out
                        logging.info(f"'H' detected in {f} at ({dx},{dy}) with a confidence of {confidence:.2f}.")
                        self.main.rxdata.cam.dump(dx, dy, self.main.rxdata.time.time)
                        await grapher.imshow(image)
                    else:
                        self.main.rxdata.cam.dump(0.0, 0.0, self.main.rxdata.time.time)
                        logging.info(f"None detected in {f}.")
        else:
            await asyncio.sleep(1 / self._freq)

    async def run(self) -> None:
        """Find and process new images."""
        logging.debug("Starting image watch cycle...")
        await self._image_processor_run_loop()


class Controller:
    """Controller class for managing MAVLINK communication with GCS.

    Attributes
    ----------
    main : 'Main'
        The Main instance to which this controller is associated.
    _txfreq : int, optional
        The transmission frequency (default is DEFAULT_FREQ).
    _heartbeatfreq : int, optional
        The heartbeat message transmission frequency (default is 1).
    _gcs_id : None
        The identifier for the GCS connection.
    _mav_conn_gcs : mavutil.mavfile
        The MAVLink connection to the GCS.

    Methods
    -------
    manager(self)
        Manage connection.
    close(self)
        Close connection.
    """

    type_handlers = {
        'HEARTBEAT': '_handle_heartbeat',
        'BAD_DATA': '_handle_bad_data',
        'CHANGE_OPERATOR_CONTROL': '_handle_change_operator_control',
        'COMMAND_LONG': '_handle_command_long',
        'COMMAND_INT': '_handle_command_int',
    }

    MAX_FLUSH_BUFFER = int(1e6)

    def __init__(self, main: 'Main', tx_freq: int = DEFAULT_FREQ, heartbeat_freq: int = 1) -> None:
        """Initialize a Controller instance.

        Parameters
        ----------
        main : 'Main'
            The Main instance to which this controller is associated.
        tx_freq : int, optional
            The transmission frequency (default is DEFAULT_FREQ).
        heartbeat_freq : int, optional
            The heartbeat message transmission frequency (default is 1).
        """

        self.main = main

        self._txfreq = tx_freq
        self._heartbeatfreq = heartbeat_freq

        self._last_cam_beat = False

        self._roi = [0.0, 0.0, 0.0] # TODO: make waypoint
        self._roi_task = None

        self._gcs_id = None
        self._mav_conn_gcs: mavutil.mavfile = mavutil.mavlink_connection(self.main.config.get('mavlink', 'uav_gcs_conn'), source_system=self.main.systemid, source_component=m.MAV_COMP_ID_AUTOPILOT1, input=False, autoreconnect=True)
        import common.key as key
        self._mav_conn_gcs.setup_signing(key.KEY.encode('utf-8'))
        self._cam_id = self.main.config.getint('main', 'cam_id')
        asyncio.create_task(self._establish_cam(key=key.CAMKEY.encode('utf-8')))

    class PreExistingConnection(Exception):
        """Exception subclass to prevent repeated MAVLINK Connection."""
        def __init__(self, message=f"A mavlink connection already exists on this node"):
            """Inits the Excpetion class with a custom message."""
            self.message = message
            super().__init__(self.message)

    @staticmethod
    def flush_buffer(connection: mavutil.mavfile) -> None:
        """Removes any MAV messages still in the connection buffer"""
        # Buffer flusher limited by MAX_FLUSH_BUFFER to avoid loop.
        try:
            for i in range(Controller.MAX_FLUSH_BUFFER):
                msg = connection.recv_match(blocking=False)
                if msg is None:
                    logging.debug(f"Buffer flushed, {i} messages cleared")
                    break
                elif i >= Controller.MAX_FLUSH_BUFFER-1:
                    logging.error(f"Messages still in buffer after {Controller.MAX_FLUSH_BUFFER} flush cycles")     
        except ConnectionError:
            logging.debug("No connection to flush.")

    async def _establish_cam(self, key: bytes, timeout_cycles: int = 5):
        try:
            self._cam_conn: mavutil.mavfile = mavutil.mavlink_connection(self.main.config.get('mavlink', 'uav_camera_conn'), source_system=self.main.systemid, source_component=m.MAV_COMP_ID_AUTOPILOT1, input=True)
            self._cam_conn.setup_signing(key)

            Controller.flush_buffer(self._cam_conn)

            logging.info(f"Connecting to camera #{self._cam_id}...")

            self._cam_conn.mav.change_operator_control_send(self._cam_id, 0, 0, key)
            await asyncio.sleep(1)

            while not self.main.stop.is_set():
                try:
                    msg = self._cam_conn.recv_msg()
                except ConnectionResetError:
                    raise Controller.PreExistingConnection

                if msg is not None and msg.get_type()=='CHANGE_OPERATOR_CONTROL_ACK':# and msg.get_srcSystem()==target and msg.gcs_system_id==self.main.systemid:
                    match msg.ack:
                        case 0:
                            logging.info(f"Connected to camera #{self._cam_id}")
                            break
                        case 1 | 2:
                            logging.info(f"Bad connection key for camera #{self._cam_id}")
                            break
                        case 3:
                            logging.info(f"Camera #{self._cam_id} is connected to another UAV")
                            break

                self._cam_conn.mav.change_operator_control_send(self._cam_id, 0, 0, key)
                await asyncio.sleep(0)
        except KeyboardInterrupt:
            pass

    #region Handlers
    def _handle_heartbeat(self, msg) -> None:
        """Handle HEARTBEAT messages."""
        logging.debug(f"Heartbeat message from link #{msg.get_srcSystem()}")

    def _handle_bad_data(self, msg) -> None:
        """Handle BAD_DATA messages."""
        pass

    def _handle_change_operator_control(self, msg) -> None:
        """Handle CHANGE_OPERATOR_CONTROL messages."""
        if msg.target_system == self.main.systemid:
            import common.key as key
            if msg.control_request == 0 and msg.passkey==key.KEY:
                if self._gcs_id is None or self._gcs_id==msg.get_srcSystem():
                    # Accepted
                    if self._gcs_id!=msg.get_srcSystem():
                        logging.info(f"Accepting control request from GCS ({msg.get_srcSystem()})")
                    self._gcs_id = msg.get_srcSystem()
                    self._mav_conn_gcs.mav.change_operator_control_ack_send(msg.get_srcSystem(), msg.control_request, 0)
                else:
                    # Already controlled
                    logging.info(f"Rejecting second control request from {msg.get_srcSystem()}")
                    self._mav_conn_gcs.mav.change_operator_control_ack_send(msg.get_srcSystem(), msg.control_request, 3)
            elif msg.control_request == 1 and msg.passkey==key.KEY:# and self._gcs_id is not None:
                # Accepted (released)
                logging.info(f"Releasing from GCS ({self._gcs_id})")
                self._gcs_id = None
                self._mav_conn_gcs.mav.change_operator_control_ack_send(msg.get_srcSystem(), msg.control_request, 0)
            else:
                # Bad key
                logging.info(f"Bad key in GCS control request")
                self._mav_conn_gcs.mav.change_operator_control_ack_send(msg.get_srcSystem(), msg.control_request, 1)

    def _handle_command_long(self, msg) -> None:
        """Handle COMMAND_LONG messages."""
        if msg.target_system==self.main.systemid and msg.get_srcSystem()==self._gcs_id:
            match msg.command:
                # DO_SET_MODE
                case m.MAV_CMD_DO_SET_MODE:
                    logging.info("Mode change requested")
                        
                    if self.main.state.set_mode(msg.param1, msg.param2, msg.param3): # TODO: Add safety check to verify message like below
                        self._mav_conn_gcs.mav.command_ack_send(m.MAV_CMD_DO_SET_MODE, m.MAV_RESULT_ACCEPTED, 255, 0, 0, 0)
                    else:
                        self._mav_conn_gcs.mav.command_ack_send(m.MAV_CMD_DO_SET_MODE, m.MAV_RESULT_DENIED, 255, 0, 0, 0)
                # DO_CHANGE_ALTITUDE
                case m.MAV_CMD_DO_CHANGE_ALTITUDE:
                    pass # TODO
                    try:
                        self.main.processor.spf_altitude = msg.param1 # TODO add checks!
                        logging.info(f"GCS commanded altitude setpoint to {msg.param1}")
                        self._mav_conn_gcs.mav.command_ack_send(m.MAV_CMD_DO_CHANGE_ALTITUDE, m.MAV_RESULT_ACCEPTED, 255, 0, 0, 0)
                    except AttributeError:
                        self._mav_conn_gcs.mav.command_ack_send(m.MAV_CMD_DO_CHANGE_ALTITUDE, m.MAV_RESULT_TEMPORARILY_REJECTED, 255, 0, 0, 0)
                # DO_CHANGE_SPEED
                case m.MAV_CMD_DO_CHANGE_SPEED:
                    pass # TODO
                    try:
                        self.main.processor.spf_ias = msg.param2 # TODO add checks!
                        logging.info(f"GCS commanded speed setpoint to {msg.param2}")
                        self._mav_conn_gcs.mav.command_ack_send(m.MAV_CMD_DO_CHANGE_SPEED, m.MAV_RESULT_ACCEPTED, 255, 0, 0, 0)
                    except AttributeError:
                        self._mav_conn_gcs.mav.command_ack_send(m.MAV_CMD_DO_CHANGE_SPEED, m.MAV_RESULT_TEMPORARILY_REJECTED, 255, 0, 0, 0)
                # DO_GIMBAL_MANAGER_PITCHYAW
                case m.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW:
                    logging.info(f"GCS commanding camera to pitch:{msg.param1}, yaw: {msg.param2}")
                    print(euler_to_quaternion(0.0, math.radians(msg.param1), math.radians(msg.param2)))
                    self._cam_conn.mav.gimbal_device_set_attitude_send(
                        self._cam_id,
                        m.MAV_COMP_ID_CAMERA,
                        m.GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME,
                        euler_to_quaternion(0.0, math.radians(msg.param1), math.radians(msg.param2)), # TODO: q values wxyz
                        0.0, 0.0, 0.0 # angular velocities
                    )
                    self._cam_conn.mav.command_ack_send(m.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW, m.MAV_RESULT_ACCEPTED, 255, 0, 0, 0)
                # DO_GIMBAL_SET_ROI_NONE
                case m.MAV_CMD_DO_SET_ROI_NONE:
                    logging.info(f"GCS commanding camera to reset ROI")
                    if self._roi_task is not None:
                        self._roi_task.cancel()
                        self._roi_task = None
                    self._cam_conn.mav.gimbal_device_set_attitude_send(
                        self._cam_id,
                        m.MAV_COMP_ID_CAMERA,
                        m.GIMBAL_DEVICE_FLAGS_NEUTRAL,
                        [1.0, 0.0, 0.0, 0.0],
                        0.0, 0.0, 0.0
                    )
                    self._cam_conn.mav.command_ack_send(m.MAV_CMD_DO_SET_ROI_NONE, m.MAV_RESULT_ACCEPTED, 255, 0, 0, 0)
                # Command unsupported
                case _:
                    self._mav_conn_gcs.mav.command_ack_send(msg.command, m.MAV_RESULT_UNSUPPORTED, 255, 0, 0, 0)

    def _handle_command_int(self, msg) -> None:
        """Handle COMMAND_INT messages."""
        if msg.target_system==self.main.systemid and msg.get_srcSystem()==self._gcs_id:
            match msg.command:
                # DO_REPOSITION
                case m.MAV_CMD_DO_REPOSITION:
                    self.main.navigator.next_wpt(
                        Waypoint(msg.x / 1e7, msg.y / 1e7, msg.z) if msg.z > 0.1 else Waypoint(msg.x / 1e7, msg.y / 1e7)
                    )

                    logging.info(f"Directing to {self.main.navigator._waypoint_list[1].latitude}, {self.main.navigator._waypoint_list[1].longitude}")
                # TODO TEMPORARY PID
                case 0:
                    # PID TUNER GOTO
                    self.main.processor._pidv_rls_out.reset()
                    self.main.processor._pidv_pts_out.reset()
                    self.main.processor._pidv_rol_rls.reset()
                    self.main.processor._pidv_pit_pts.reset()
                    # self.main.processor._pidv_pts_out.set(kp=msg.param1, ti=msg.param2, td=msg.param3)
                    self.main.processor._pidv_pts_out.set(kp=msg.param1, ti=msg.param2, td=msg.param3)
                    # self.main.processor._pidv_pit_pts.set(kp=msg.param1, ti=msg.param2, td=msg.param3)
                    self.main.processor._pidv_pit_pts.set(kp=msg.param4)
                    # self.main.processor._spv_pitchspeed=msg.param4
                    logging.info(f"New PID state: {msg.param1}, {msg.param2}, {msg.param3} @ {msg.param4}")
                # TODO TEMPORARY SCREENSHOT
                case 1:
                    logging.info("Commanding camera...")
                    self._cam_conn.mav.command_long_send(
                        self._cam_id,
                        m.MAV_COMP_ID_CAMERA,
                        m.MAV_CMD_IMAGE_START_CAPTURE,
                        0,
                        0, # param1: Camera ID
                        msg.param1, # param2: Interval (seconds)
                        msg.param2, # param3: Number images
                        msg.param3, # param4: Single image sequence number
                        0,
                        0,
                        float('nan')
                    )
                # NAV_TAKEOFF
                case m.MAV_CMD_NAV_TAKEOFF:
                    pass # TODO
                # NAV_VTOL_TAKEOFF
                case m.MAV_CMD_NAV_VTOL_TAKEOFF:
                    pass # TODO
                # NAV_LAND
                case m.MAV_CMD_NAV_LAND:
                    pass # TODO
                # NAV_VTOL_LAND
                case m.MAV_CMD_NAV_VTOL_LAND:
                    pass # TODO
                # DO_GIMBAL_SET_ROI_LOCATION
                case m.MAV_CMD_DO_SET_ROI_LOCATION:
                    lat = msg.x / 1e7
                    lon = msg.y / 1e7
                    alt = msg.z
                    logging.info(f"GCS commanding camera to lat:{lat}, lon: {lon}, alt: {alt}")
                    self._roi = [lat, lon, alt]
                    if self._roi_task is None:
                        self._roi_task = asyncio.create_task(self._roi_calc())
                    self._cam_conn.mav.command_ack_send(m.MAV_CMD_DO_SET_ROI_LOCATION, m.MAV_RESULT_ACCEPTED, 255, 0, 0, 0)
                # Command unsupported
                case _:
                    self._mav_conn_gcs.mav.command_ack_send(msg.command, m.MAV_RESULT_UNSUPPORTED, 255, 0, 0, 0)
    #endregion
    
    @async_loop_decorator()
    async def _manager_loop(self) -> None:
        await asyncio.sleep(0)

    async def manager(self) -> None:
        """Manage the controller's various operations."""
        asyncio.create_task(self._heartbeat())
        asyncio.create_task(self._rx())
        asyncio.create_task(self._rxcam())
        asyncio.create_task(self._tx())

        logging.info("Starting Controller")

        await self._manager_loop()

    @async_loop_decorator(close=False)
    async def _controller_rx_loop(self) -> None:
        """Recieve messages from GCS over mavlink."""
        try:
            msg = self._mav_conn_gcs.recv_msg()
        except ConnectionError:
            try:
                logging.debug("Controller (rx) connection refused")
                await asyncio.sleep(0)
            except asyncio.exceptions.CancelledError:
                self.main.stop.set()
                raise
            finally:
                return

        if msg is not None:
            type_ = msg.get_type()
            if type_ in Controller.type_handlers:
                handler = getattr(self, Controller.type_handlers[type_])
                handler(msg)
            else:
                logging.info(f"Unknown message type: {type_}")

        await asyncio.sleep(0)

    @async_loop_decorator(close=False)
    async def _controller_rxcam_loop(self) -> None:
        """Recieve messages from GCS over mavlink."""
        try:
            msg = self._cam_conn.recv_msg()
        except ConnectionError:
            logging.debug("No connection to listen to.")
            return
        except OSError:
            logging.debug("No connection to listen to.")
            return

        target = self.main.config.getint('main', 'cam_id')

        
        if msg is not None:
            if msg.get_type() == 'HEARTBEAT' and msg.get_srcSystem()==target:
                logging.debug(f"Heartbeat message from camera #{msg.get_srcSystem()}")
                self._last_cam_beat = self.main.rxdata.time.time

        if self._last_cam_beat:
            if self.main.rxdata.time.time-self._last_cam_beat > HEARTBEAT_TIMEOUT*1e6: # TODO: remove time module dependency
                logging.warning(f"Heartbeat timeout from camera #{target}, closing...")
                self._last_cam_beat = False
                self._cam_conn.close()
                import common.key as key
                asyncio.create_task(self._establish_cam(key=key.CAMKEY.encode('utf-8')))

        await asyncio.sleep(0)

    async def _rx(self) -> None:
        """Recieve messages from GCS over mavlink."""
        logging.debug("Starting Controller (RX)")
        await self._controller_rx_loop()

    async def _rxcam(self) -> None:
        """Recieve messages from camera over mavlink."""
        logging.debug("Starting camera (RX)")
        await self._controller_rxcam_loop()

    @async_loop_decorator(close=False)
    async def _controller_tx_loop(self) -> None:
        """Transmit continuous messages."""
        await asyncio.sleep(0)

    async def _tx(self) -> None:
        """Transmit continuous messages."""
        logging.debug("Starting Controller (TX)")
        await self._controller_tx_loop()

    async def _roi_calc(self) -> None:
        try:
            logging.info("Starting ROI cycle")
            while not self.main.stop.is_set():
                try:
                    await asyncio.sleep(0)
                    y, p, _ = gps_angles(
                        self.main.rxdata.gps.latitude,
                        self.main.rxdata.gps.longitude,
                        self.main.rxdata.gps.altitude,
                        *self._roi
                    )
                    y = calc_dyaw(self.main.rxdata.att.yaw, math.radians(y))
                    p = self.main.rxdata.att.pitch-math.radians(p)
                    # print(f"slewing to {p}, {y}")
                    quat = euler_to_quaternion(0.0, p, y)
                    self._cam_conn.mav.gimbal_device_set_attitude_send(
                        self._cam_id,
                        m.MAV_COMP_ID_CAMERA,
                        m.GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME,
                        quat, # TODO: q values wxyz
                        0.0, 0.0, 0.0 # angular velocities
                    )
                    await asyncio.sleep(1 / self._txfreq)
                except asyncio.exceptions.CancelledError:
                    break
                except Exception as e:
                    logging.error(f"Error in ROI cycle: {e}")
        finally:
            logging.info("Closing ROI cycle")

    @async_loop_decorator(close=False)
    async def _controller_heartbeat_loop(self) -> None:
        msg = [
            m.MAV_TYPE_VTOL_RESERVED4, # 24
            m.MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY,
            int(self.main.state.mode),
            int(self.main.state.custom_mode),
            int(self.main.state.state)
        ]

        self._mav_conn_gcs.mav.heartbeat_send(*msg)
        self._cam_conn.mav.heartbeat_send(*msg)
        
        logging.debug("TX Heartbeat")
        await asyncio.sleep(1 / self._heartbeatfreq)

    async def _heartbeat(self) -> None:
        """Periodically publish a heartbeat message."""
        logging.debug("Starting Controller (Heartbeat)")
        await self._controller_heartbeat_loop()

    def close(self) -> None:
        """Close the instance."""
        self._mav_conn_gcs.close()

        import common.key as key
        self._cam_conn.mav.change_operator_control_send(self.main.config.getint('main', 'cam_id'), 1, 0, key.CAMKEY.encode('utf-8'))
        self._cam_conn.close()
        logging.info("Closing Controller")


class Main:
    """Main class for handling the operation of a system.

    Parameters
    ----------
    systemid : int, optional
        The system ID for this instance (default is 1).
    config : str, optional
        Config file path (default is './common/CONFIG.ini').

    Attributes
    ----------
    systemid : int
        The system ID.
    config : ConfigParser
        The configuration parser.
    boot : asyncio.Event
        An event to signal the system boot.
    stop : asyncio.Event
        An event to signal the system to stop.
    rxdata : GlobalRx
        Global receive data.
    txdata : GlobalTx
        Global transmit data.
    state : g
        State information.
    
    Methods
    -------
    run(self, graph: str = None)
        Run the UAV instance.
    """

    def __init__(self, config: str = './common/CONFIG.ini') -> None:
        """Initialize a Main instance.

        Parameters
        ----------
        systemid : int, optional
            The system ID for this instance (default is 1).
        config : str, optional
            The path to the configuration file (default is './common/CONFIG.ini').

        Raises
        ------
        AssertionError
            If the system ID is not a positive integer <= 255.
        """

        self.config = ConfigParser()
        self.config.read(config)

        self.systemid = self.config.getint('main', 'uav_id')
        assert isinstance(self.systemid, int) and self.systemid > 0 and self.systemid.bit_length() <= 8, "System ID in config file must be UINT8"

        self.boot = asyncio.Event()
        self.stop = asyncio.Event()

        self.rxdata = GlobalRx()
        self.txdata = GlobalTx()

        self.state = g(m.MAV_STATE_UNINIT, m.MAV_MODE_PREFLIGHT, g.CUSTOM_MODE_UNINIT, g.CUSTOM_SUBMODE_UNINIT, self.boot)

    async def _graph(self, name: str = '0.0', freq: int = 10) -> None:
        """Asynchronously collect and graph data.

        Parameters
        ----------
        name : str, optional
            The name of the data variable to graph (default is '0.0').
        freq : int, optional
            The graph update frequency in Hz (default is 10).
        """

        self._grapher = grapher.Grapher(deque_len=300)

        try:
            while not self.stop.is_set():
                try:
                    self._grapher.add(eval('self.' + name))
                    self._grapher.graph()

                    try:
                        await asyncio.sleep(1 / freq)
                    except asyncio.exceptions.CancelledError:
                        self.stop.set()
                        raise
                except Exception as e:
                    logging.error(f'Error in Grapher: {e}')
        except KeyboardInterrupt:
            self.stop.set()
            raise
        finally:
            self._grapher.close()
            logging.info('Closing Grapher')

    async def _print(self, name: str = 'Nothing to print.', freq: int = 10) -> None:
            """Asynchronously collect and print data.

            Parameters
            ----------
            name : str, optional
                The name of the data variable to graph (default is 'Nothing to print.').
            freq : int, optional
                The terminal update frequency in Hz (default is 10).
            """

            try:
                while not self.stop.is_set():
                    try:
                        print(eval('self.' + name))

                        try:
                            await asyncio.sleep(1 / freq)
                        except asyncio.exceptions.CancelledError:
                            self.stop.set()
                            raise
                    except Exception as e:
                        logging.error(f'Error in Printer: {e}')
            except KeyboardInterrupt:
                self.stop.set()
                raise

    async def run(self, graph: str | bool = False, print_: str | bool = False) -> None:
        """Run the main system components.

        Leaving graph set to default will not start a graphing window.

        Parameters
        ----------
        graph : str | None, optional
            The name of the variable to graph (default is None).
        """

        self.controller = Controller(self)
        self.io = MainIO(self)
        self.processor = Processor(self)
        self.navigator = Navigator(self)
        self.img = ImageProcessor(self)

        controller_manager = asyncio.create_task(self.controller.manager())
        await asyncio.sleep(0)

        logging.warning(f"Creating instance #{self.systemid}, waiting for boot command from GCS")

        try:
            self.boot.set()
            self.state.inc_mode()
            await self.boot.wait()
        except asyncio.exceptions.CancelledError:
            controller_manager.cancel()
            await asyncio.sleep(0)
            logging.warning(f"Never booted, closing instance #{self.systemid}")
            sys.exit()

        logging.warning(f"Booting instance #{self.systemid}...")

        boot_tasks = [
            asyncio.create_task(self.processor.boot()),
            asyncio.create_task(self.navigator.boot()),
            asyncio.create_task(self.io.boot()),
        ]

        try:
            await asyncio.gather(*boot_tasks)
        except asyncio.exceptions.CancelledError:
            controller_manager.cancel()
            await asyncio.sleep(0)
            logging.warning(f"Ctrl-C during boot cycle, closing instance #{self.systemid}")
            sys.exit()

        logging.warning(f"Boot successful on #{self.systemid}")

        for _ in range(_DEBUG_SKIP):
            self.state.inc_mode()

        tasks = [
            asyncio.create_task(self.processor.run()),
            asyncio.create_task(self.navigator.run()),
            asyncio.create_task(self.io.run()),
            asyncio.create_task(self.img.run()),
        ]

        if graph:
            logging.info("Grapher on")
            tasks.append(asyncio.create_task(self._graph(name=graph)))

        if print_:
            logging.info("Printer on")
            tasks.append(asyncio.create_task(self._print(name=print_)))

        try:
            await asyncio.gather(*tasks)
        except asyncio.exceptions.CancelledError:
            self.stop.set()

        logging.warning(f"Closing instance #{self.systemid}")


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-g", "--graph", nargs='?', default=False, const='rxdata.att.rollspeed', help="Attribute to graph")
    parser.add_argument("-p", "--print", nargs='?', default=False, const='processor._throttles', help="Attribute to print")
    parser.add_argument("-s", "--skip", nargs='?', default=False, const='2', help="Skip number of modes on startup")
    args = parser.parse_args()

    whitelisted = frozenset("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789._")
    if args.graph:
        assert all(char in whitelisted for char in args.graph) and '__' not in args.graph
    if args.print:
        assert all(char in whitelisted for char in args.print) and '__' not in args.print

    _DEBUG_SKIP = int(args.skip)

    main = Main()
    asyncio.run(main.run(graph=args.graph, print_=args.print))
