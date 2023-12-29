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
import json
import logging
import math
import os
import sys
from configparser import ConfigParser

import numpy as np

os.chdir(os.path.dirname(os.path.realpath(__file__)) + '/..')
sys.path.append(os.getcwd())
for var in os.environ:
    if var.startswith('UAVCAN__'):
        os.environ.pop(var)
os.environ['CYPHAL_PATH'] = './common/public_regulated_data_types'
os.environ['PYCYPHAL_PATH'] = './common/pycyphal_generated'
os.environ['UAVCAN__DIAGNOSTIC__SEVERITY'] = '2'
os.environ['MAVLINK20'] = '1'
os.environ['MAVLINK_DIALECT'] = 'common'

filehandler = logging.FileHandler('uav/uav.log', mode='a')
filehandler.setLevel(logging.INFO)
filehandler.setFormatter(logging.Formatter(str(os.getpid()) + ' (%(asctime)s %(name)s) %(levelname)s:%(message)s'))
streamhandler = logging.StreamHandler()
streamhandler.setLevel(logging.INFO)
streamhandler.setFormatter(logging.Formatter('(%(name)s)  %(levelname)s:%(message)s'))
logger = logging.getLogger("UAV")
logger.addHandler(filehandler)
logger.addHandler(streamhandler)
logger.setLevel(logging.DEBUG)

MAVLOG_DEBUG = logging.DEBUG - 3
MAVLOG_TX = logging.DEBUG - 2
MAVLOG_RX = logging.DEBUG - 1
MAVLOG_LOG = logging.INFO + 1

logger.warning("Generating UAVCAN files, please wait...")

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
from common.decorators import async_loop_decorator
from common.pid import PID
from common.states import GlobalStates as g
from common.states import NodeCommands
from common.angles import quaternion_to_euler, euler_to_quaternion, gps_angles, calc_dyaw

m = mavutil.mavlink

for name in ['pymavlink', 'pycyphal', 'pydsdl', 'nunavut', 'matplotlib']:
    logging.getLogger(name).setLevel(logging.WARNING)
filehandler.setLevel(logging.DEBUG)

db_config = ConfigParser()
db_config.read('./common/_db_config.ini')

DEFAULT_FREQ = 50
HEARTBEAT_TIMEOUT = 2.0

RPM_TO_RADS = math.pi/30
KT_TO_MS = 0.514444

DEBUG_SKIP = -1 # TODO: remove this!!

system_ids = []


class StateManager:
    def __init__(self, state: int, mode: int, custom_mode: int, custom_submode: int, boot: asyncio.Event) -> None:
        """Inits the state manager."""
        state = int(state)
        mode = int(mode)
        custom_mode = int(custom_mode)
        custom_submode = int(custom_submode)

        assert state in g.MAV_STATES_NOMINAL or state in g.MAV_STATES_ABNORMAL, "g input 'state' must be MAV_STATE int"
        assert mode in g.MAV_MODES, "g input 'mode' must be MAV_MODE int"
        assert custom_mode in g.CUSTOM_MODES, "g input 'custom_mode' must be g.CUSTOM_MODE int"
        assert custom_submode in g.CUSTOM_SUBMODES, "g input 'custom_submode' must be g.CUSTOM_SUBMODE int"
        
        self.state = state
        self.mode = mode
        self.custom_mode = custom_mode
        self.custom_submode = custom_submode
        logger.warning(f"Submode set to: {g.CUSTOM_SUBMODE_NAMES[self.custom_submode]}")

        self.boot = boot

    def set_mode(self, mode: int, custom_mode: int, custom_submode: int) -> bool:
        """Sets a new custom_mode and checks for compatibility."""
        try:
            mode = int(mode)
            custom_mode = int(custom_mode)
            custom_submode = int(custom_submode)

            if mode == m.MAV_MODE_MANUAL_ARMED and self.custom_mode == g.CUSTOM_MODE_FLIGHT:
                self.custom_submode = g.CUSTOM_SUBMODE_FLIGHT_MANUAL
                logger.warning(f"Submode set to: {g.CUSTOM_SUBMODE_NAMES[self.custom_submode]}")
                self.mode = mode
                self.state = m.MAV_STATE_ACTIVE
                return True

            if self.custom_submode == g.CUSTOM_SUBMODE_UNINIT and custom_submode == g.CUSTOM_SUBMODE_BOOT:
                self.boot.set()

            if custom_submode != self.custom_submode:
                assert custom_submode in g.ALLOWED_SUBMODE_CHANGES[self.custom_submode]
                self.custom_submode = custom_submode
                logger.warning(f"Submode set to: {g.CUSTOM_SUBMODE_NAMES[self.custom_submode]}")

                if self.state not in g.MAV_STATES_ABNORMAL:
                    self.state = g.ALLOWED_STATES[self.custom_submode][0]

            if mode != self.mode:
                if mode in g.ALLOWED_MODES[self.custom_submode]:
                    self.mode = mode
                else:
                    self.mode = g.ALLOWED_MODES[self.custom_submode][0]

            if custom_mode != self.custom_mode:
                if custom_mode in g.ALLOWED_CUSTOM_MODES[self.custom_submode]:
                    self.custom_mode = custom_mode
                else:
                    self.custom_mode = g.ALLOWED_CUSTOM_MODES[self.custom_submode][0]

            return True
        except AssertionError:
            return False

    def inc_mode(self) -> None:
        """Steps the mode up one in the standard flight sequence."""
        self.custom_submode = g.ALLOWED_SUBMODE_CHANGES[self.custom_submode][0]
        logger.warning(f"Submode set to: {g.CUSTOM_SUBMODE_NAMES[self.custom_submode]}")
        self.mode = g.ALLOWED_MODES[self.custom_submode][0]
        self.custom_mode = g.ALLOWED_CUSTOM_MODES[self.custom_submode][0]

        if self.state not in g.MAV_STATES_ABNORMAL:
            self.state = g.ALLOWED_STATES[self.custom_submode][0]

    def dec_mode(self) -> None:
        """Steps the mode down one in the standard flight sequence."""
        if self.custom_submode not in [g.CUSTOM_SUBMODE_FLIGHT_MANUAL, g.CUSTOM_SUBMODE_FLIGHT_TERRAIN_AVOIDANCE, g.CUSTOM_SUBMODE_UNINIT]:
            self.custom_submode = g.ALLOWED_SUBMODE_CHANGES[self.custom_submode][1]
            logger.warning(f"Submode set to: {g.CUSTOM_SUBMODE_NAMES[self.custom_submode]}")
            self.mode = g.ALLOWED_MODES[self.custom_submode][0]
            self.custom_mode = g.ALLOWED_CUSTOM_MODES[self.custom_submode][0]

            if self.state not in g.MAV_STATES_ABNORMAL:
                self.state = g.ALLOWED_STATES[self.custom_submode][0]
        else:
            logger.debug("Invalid submode for StateManager.dec_mode(), running StateManager.inc_mode() instead")
            self.inc_mode()

    def set_state(self, state: int) -> None:
        """Sets a new MAVLINK state."""
        if (state in g.MAV_STATES_NOMINAL or state in g.MAV_STATES_ABNORMAL) and state in g.ALLOWED_STATES[self.custom_submode]:
            self.state = state
        else:
            logger.debug("Invalid state passed into StateManager.set_state()")


class RxBuffer:
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
            _nspeed, _espeed, _dspeed = msg.value.twist.value.linear.meter_per_second

            self.yspeed = _nspeed*math.cos(self.yaw) + _espeed*math.sin(self.yaw)
            self.xspeed = -_nspeed*math.sin(self.yaw) + _espeed*math.cos(self.yaw)
            self.zspeed = -1*_dspeed
            self.dt = self.time - self._last_time

    class Alt:
        """Store altimeter data."""
        def __init__(self) -> None:
            self.time = 0.0
            self.altitude = 0.0

            self._last_time = 0.0
            self.dt = 0.0
            self._last_altitude = 0.0
            self._last_vs = 0.0

        def dump(self, msg: uavcan.si.unit.length.WideScalar_1, time: 'RxBuffer.Time.time') -> None:
            """Store data from a message."""
            self._last_time = self.time
            self._last_altitude = self.altitude

            self.time = time
            self.altitude = msg.meter

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

        def dump(self, msg: reg.udral.physics.kinematics.geodetic.PointStateVarTs_0, heading: 'RxBuffer.Att.yaw') -> None:
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
            self.altitude = msg.value.position.value.altitude.meter
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

        def dump(self, msg: uavcan.si.unit.angle.Scalar_1, time: 'RxBuffer.Time.time') -> None:
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

        def dump(self, xdp: float, ydp: float, time: 'RxBuffer.Time.time') -> None:
            """Store data from a message."""
            self._last_time = self.time
            # self._last_aoa = self.aoa
            
            self.time = time
            self.xdp = xdp
            self.ydp = ydp

            self.dt = self.time - self._last_time

    def __init__(self) -> None:
        self.time = RxBuffer.Time()
        self.att =  RxBuffer.Att()
        self.alt =  RxBuffer.Alt()
        self.gps =  RxBuffer.Gps()
        self.ias =  RxBuffer.Ias()
        self.aoa =  RxBuffer.Aoa()
        self.cam =  RxBuffer.Cam()


class TxBuffer:
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


class UAVCANManager:
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
        Close the UAVCANManager node and release resources.
    """

    class NodeManager:
        """Find node ids corresponding with critical components."""
        MOTORHUB_NAME = 'fmuas.motorhub'
        SENSORHUB_NAME = 'fmuas.sensorhub'
        GPS_NAME = 'fmuas.gps'
        CLOCK_NAME = 'fmuas.clock'

        class Node:
            defaults = (0, pycyphal.application.node_tracker.Entry(uavcan.node.Heartbeat_1(), None).heartbeat)
            def __init__(self) -> None:
                self.id: int
                self.heartbeat: uavcan.node.Heartbeat_1
                self.active = asyncio.Event()
                self.id, self.heartbeat = UAVCANManager.NodeManager.Node.defaults

            def set(self, heartbeat: uavcan.node.Heartbeat_1, id: int = None,) -> None:
                self.id = id if id is not None else self.id
                self.heartbeat = heartbeat
                self.active.set()

            def reset(self) -> None:
                self.id, self.heartbeat = UAVCANManager.NodeManager.Node.defaults
                self.active.clear()

        def __init__(self, node: pycyphal.application.Node, config: ConfigParser | None = None) -> None:
            # If a config is passed, fixed node ids will be used.
            self.clock = UAVCANManager.NodeManager.Node()
            self.sensorhub = UAVCANManager.NodeManager.Node()
            self.motorhub = UAVCANManager.NodeManager.Node()
            self.gps = UAVCANManager.NodeManager.Node()
            
            if config:
                self.clock.id = config.getint('node_ids', 'clock')
                self.sensorhub.id = config.getint('node_ids', 'sensorhub')
                self.motorhub.id = config.getint('node_ids', 'motorhub')
                self.gps.id = config.getint('node_ids', 'gps')
            
            self.node = node

            self._sub_heartbeat = self.node.make_subscriber(uavcan.node.Heartbeat_1)

            def update_heartbeat(msg: uavcan.node.Heartbeat_1, info: pycyphal.transport.TransferFrom) -> None:
                if info.source_node_id == self.clock.id:
                    self.clock.set(msg)
                elif info.source_node_id == self.sensorhub.id:
                    self.sensorhub.set(msg)
                elif info.source_node_id == self.motorhub.id:
                    self.motorhub.set(msg)
                elif info.source_node_id == self.gps.id:
                    self.gps.set(msg)
            self._sub_heartbeat.receive_in_background(update_heartbeat)

        def update(self, id: int, old: pycyphal.application.node_tracker.Entry | None, new: pycyphal.application.node_tracker.Entry | None) -> None:
            if new is None: # node offline
                self._destroy_values(id)
            elif new.info is not None: # node has info
                self._update_values(id, new)

        def _update_values(self, id: int, entry: pycyphal.application.node_tracker.Entry) -> None:
            name = ''.join(chr(c) for c in entry.info.name)
            if id==self.clock.id or name==UAVCANManager.NodeManager.CLOCK_NAME:
                self.clock.set(entry.heartbeat, id)
            elif id==self.sensorhub.id or name==UAVCANManager.NodeManager.SENSORHUB_NAME:
                self.sensorhub.set(entry.heartbeat, id)
            elif id==self.motorhub.id or name==UAVCANManager.NodeManager.MOTORHUB_NAME:
                self.motorhub.set(entry.heartbeat, id)
            elif id==self.gps.id or name==UAVCANManager.NodeManager.GPS_NAME:
                self.gps.set(entry.heartbeat, id)
        
        def _destroy_values(self, id: int) -> None:
            if id==self.clock.id:
                self.clock.reset()
            elif id==self.sensorhub.id:
                self.sensorhub.reset()
            elif id==self.motorhub.id:
                self.motorhub.reset()
            elif id==self.gps.id:
                self.gps.reset()

    def __init__(self, main: 'Main', freq: int = DEFAULT_FREQ) -> None:
        """Initialize the UAVCANManager class.

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

        if os.path.exists(f:='./'+self.main.config.get('db_files', 'uavmain')):
            os.remove(f)
            logger.debug(f"Removing preexisting {f}")
        logger.debug(f"Compiling {f}")

        _registry = pycyphal.application.make_registry(environment_variables={
            'UAVCAN__NODE__ID'                      :db_config.get('node_ids', 'uavmain'),
            'UAVCAN__UDP__IFACE'                    :db_config.get('main', 'udp'),

            'UAVCAN__PUB__SERVO_READINESS__ID'      :db_config.get('subject_ids', 'servo_readiness'),
            'UAVCAN__PUB__ESC_READINESS__ID'        :db_config.get('subject_ids', 'esc_readiness'),

            'UAVCAN__PUB__ELEVON1_SP__ID'           :db_config.get('subject_ids', 'elevon1_sp'),
            'UAVCAN__SUB__ELEVON1_FEEDBACK__ID'     :db_config.get('subject_ids', 'elevon1_feedback'),
            'UAVCAN__SUB__ELEVON1_STATUS__ID'       :db_config.get('subject_ids', 'elevon1_status'),
            'UAVCAN__SUB__ELEVON1_POWER__ID'        :db_config.get('subject_ids', 'elevon1_power'),
            'UAVCAN__SUB__ELEVON1_DYNAMICS__ID'     :db_config.get('subject_ids', 'elevon1_dynamics'),

            'UAVCAN__PUB__ELEVON2_SP__ID'           :db_config.get('subject_ids', 'elevon2_sp'),
            'UAVCAN__SUB__ELEVON2_FEEDBACK__ID'     :db_config.get('subject_ids', 'elevon2_feedback'),
            'UAVCAN__SUB__ELEVON2_STATUS__ID'       :db_config.get('subject_ids', 'elevon2_status'),
            'UAVCAN__SUB__ELEVON2_POWER__ID'        :db_config.get('subject_ids', 'elevon2_power'),
            'UAVCAN__SUB__ELEVON2_DYNAMICS__ID'     :db_config.get('subject_ids', 'elevon2_dynamics'),

            'UAVCAN__PUB__TILT_SP__ID'              :db_config.get('subject_ids', 'tilt_sp'),
            'UAVCAN__SUB__TILT_FEEDBACK__ID'        :db_config.get('subject_ids', 'tilt_feedback'),
            'UAVCAN__SUB__TILT_STATUS__ID'          :db_config.get('subject_ids', 'tilt_status'),
            'UAVCAN__SUB__TILT_POWER__ID'           :db_config.get('subject_ids', 'tilt_power'),
            'UAVCAN__SUB__TILT_DYNAMICS__ID'        :db_config.get('subject_ids', 'tilt_dynamics'),

            'UAVCAN__PUB__ESC1_SP__ID'              :db_config.get('subject_ids', 'esc1_sp'),
            'UAVCAN__SUB__ESC1_FEEDBACK__ID'        :db_config.get('subject_ids', 'esc1_feedback'),
            'UAVCAN__SUB__ESC1_STATUS__ID'          :db_config.get('subject_ids', 'esc1_status'),
            'UAVCAN__SUB__ESC1_POWER__ID'           :db_config.get('subject_ids', 'esc1_power'),
            'UAVCAN__SUB__ESC1_DYNAMICS__ID'        :db_config.get('subject_ids', 'esc1_dynamics'),

            'UAVCAN__PUB__ESC2_SP__ID'              :db_config.get('subject_ids', 'esc2_sp'),
            'UAVCAN__SUB__ESC2_FEEDBACK__ID'        :db_config.get('subject_ids', 'esc2_feedback'),
            'UAVCAN__SUB__ESC2_STATUS__ID'          :db_config.get('subject_ids', 'esc2_status'),
            'UAVCAN__SUB__ESC2_POWER__ID'           :db_config.get('subject_ids', 'esc2_power'),
            'UAVCAN__SUB__ESC2_DYNAMICS__ID'        :db_config.get('subject_ids', 'esc2_dynamics'),

            'UAVCAN__PUB__ESC3_SP__ID'              :db_config.get('subject_ids', 'esc3_sp'),
            'UAVCAN__SUB__ESC3_FEEDBACK__ID'        :db_config.get('subject_ids', 'esc3_feedback'),
            'UAVCAN__SUB__ESC3_STATUS__ID'          :db_config.get('subject_ids', 'esc3_status'),
            'UAVCAN__SUB__ESC3_POWER__ID'           :db_config.get('subject_ids', 'esc3_power'),
            'UAVCAN__SUB__ESC3_DYNAMICS__ID'        :db_config.get('subject_ids', 'esc3_dynamics'),

            'UAVCAN__PUB__ESC4_SP__ID'              :db_config.get('subject_ids', 'esc4_sp'),
            'UAVCAN__SUB__ESC4_FEEDBACK__ID'        :db_config.get('subject_ids', 'esc4_feedback'),
            'UAVCAN__SUB__ESC4_STATUS__ID'          :db_config.get('subject_ids', 'esc4_status'),
            'UAVCAN__SUB__ESC4_POWER__ID'           :db_config.get('subject_ids', 'esc4_power'),
            'UAVCAN__SUB__ESC4_DYNAMICS__ID'        :db_config.get('subject_ids', 'esc4_dynamics'),

            'UAVCAN__SUB__INERTIAL__ID'             :db_config.get('subject_ids', 'inertial'),
            'UAVCAN__SUB__ALTITUDE__ID'             :db_config.get('subject_ids', 'altitude'),
            'UAVCAN__SUB__IAS__ID'                  :db_config.get('subject_ids', 'ias'),
            'UAVCAN__SUB__AOA__ID'                  :db_config.get('subject_ids', 'aoa'),
            'UAVCAN__SUB__GPS__ID'                  :db_config.get('subject_ids', 'gps'),

            'UAVCAN__SUB__CLOCK_SYNC_TIME__ID'      :db_config.get('subject_ids', 'clock_sync_time'),
            'UAVCAN__SUB__GPS_SYNC_TIME__ID'        :db_config.get('subject_ids', 'gps_sync_time'),
        })
        
        for var in os.environ:
            if var.startswith('UAVCAN__'):
                os.environ.pop(var)
        for var, value in _registry.environment_variables.items():
            assert isinstance(value, bytes)
            os.environ[var] = value.decode('utf-8')


        self._freq = freq
        self._use_gps_time = False
        self.boot = asyncio.Event()
        
        logger.info("Initializing UAVCAN Node...")

        node_info = uavcan.node.GetInfo_1.Response(
            software_version=uavcan.node.Version_1(major=1, minor=0),
            name=f'fmuas.uavmain{self.main.systemid}',
        )

        self._registry = pycyphal.application.make_registry(self.main.config.get('db_files', 'uavmain'))
        self._node = pycyphal.application.make_node(node_info, self._registry)

        self.node_manager = UAVCANManager.NodeManager(self._node) # add config for fixed node ids
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

        self._srv_exec_cmd = self._node.get_server(uavcan.node.ExecuteCommand_1)

        # TODO: Only DECLARE (DECLARING SEPERATELY CAUSES MEM LEAK!!!) and start subs at boot
        self._sub_clock_sync_time.receive_in_background(self._on_time)
        self._sub_gps_sync_time.receive_in_background(self._on_gps_time)
        self._sub_ins.receive_in_background(self._on_att)
        self._sub_alt.receive_in_background(self._on_alt)
        self._sub_gps.receive_in_background(self._on_gps)
        self._sub_ias.receive_in_background(self._on_ias)
        self._sub_aoa.receive_in_background(self._on_aoa)
        
        self._sub_elevon1_feedback.receive_in_background(self._void_handler)
        self._sub_elevon1_status.receive_in_background(self._void_handler)
        self._sub_elevon1_power.receive_in_background(self._void_handler)
        self._sub_elevon1_dynamics.receive_in_background(self._void_handler)
        self._sub_elevon2_feedback.receive_in_background(self._void_handler)
        self._sub_elevon2_status.receive_in_background(self._void_handler)
        self._sub_elevon2_power.receive_in_background(self._void_handler)
        self._sub_elevon2_dynamics.receive_in_background(self._void_handler)
        self._sub_tilt_feedback.receive_in_background(self._void_handler)
        self._sub_tilt_status.receive_in_background(self._void_handler)
        self._sub_tilt_power.receive_in_background(self._void_handler)
        self._sub_tilt_dynamics.receive_in_background(self._void_handler)
        self._sub_esc1_feedback.receive_in_background(self._void_handler)
        self._sub_esc1_status.receive_in_background(self._void_handler)
        self._sub_esc1_power.receive_in_background(self._void_handler)
        self._sub_esc1_dynamics.receive_in_background(self._void_handler)
        self._sub_esc2_feedback.receive_in_background(self._void_handler)
        self._sub_esc2_status.receive_in_background(self._void_handler)
        self._sub_esc2_power.receive_in_background(self._void_handler)
        self._sub_esc2_dynamics.receive_in_background(self._void_handler)
        self._sub_esc3_feedback.receive_in_background(self._void_handler)
        self._sub_esc3_status.receive_in_background(self._void_handler)
        self._sub_esc3_power.receive_in_background(self._void_handler)
        self._sub_esc3_dynamics.receive_in_background(self._void_handler)
        self._sub_esc4_feedback.receive_in_background(self._void_handler)
        self._sub_esc4_status.receive_in_background(self._void_handler)
        self._sub_esc4_power.receive_in_background(self._void_handler)
        self._sub_esc4_dynamics.receive_in_background(self._void_handler)
        self._sub_clock_sync_time_last.receive_in_background(self._void_handler)
        self._sub_gps_sync_time_last.receive_in_background(self._void_handler)

        self._srv_exec_cmd.serve_in_background(self._serve_exec_cmd)

        self._node.start()

        logger.info("UAVCANManager initialized")

    async def _serve_exec_cmd(
            self,
            request: uavcan.node.ExecuteCommand_1.Request, 
            metadata: pycyphal.presentation.ServiceRequestMetadata,) -> uavcan.node.ExecuteCommand_1.Response:
        logger.debug("Execute command request %s from node %d", request, metadata.client_node_id)
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
                self.main.stop.set()
                return uavcan.node.ExecuteCommand_1.Response(
                    uavcan.node.ExecuteCommand_1.Response.STATUS_SUCCESS
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

    async def boot_proc(self) -> None:
        """Perform boot-related tasks."""
        # TODO: do boot stuff here, maybe read a different .ini?
        logger.info("Waiting for components...")
        tasks = [
            asyncio.create_task(self.node_manager.clock.active.wait()),
            asyncio.create_task(self.node_manager.sensorhub.active.wait()),
            asyncio.create_task(self.node_manager.motorhub.active.wait()),
            asyncio.create_task(self.node_manager.gps.active.wait()),
        ]
        await asyncio.gather(*tasks)
        logger.info("All components discovered")

        cln_clock_cmd = self.node_manager.node.make_client(uavcan.node.ExecuteCommand_1, self.node_manager.clock.id)
        clock_response = await cln_clock_cmd.call(uavcan.node.ExecuteCommand_1.Request(NodeCommands.BOOT))
        if clock_response is None:
            logger.error("CLOCK failed to respond to boot command")
        elif clock_response[0].status != uavcan.node.ExecuteCommand_1.Response.STATUS_SUCCESS:
            logger.error("CLOCK failed to respond to boot command")
        else:
            logger.debug("CLOCK powered off")

        cln_sensorhub_cmd = self.node_manager.node.make_client(uavcan.node.ExecuteCommand_1, self.node_manager.sensorhub.id)
        sensorhub_response = await cln_sensorhub_cmd.call(uavcan.node.ExecuteCommand_1.Request(NodeCommands.BOOT))
        if sensorhub_response is None:
            logger.error("SENSORHUB failed to respond to boot command")
        elif sensorhub_response[0].status != uavcan.node.ExecuteCommand_1.Response.STATUS_SUCCESS:
            logger.error("SENSORHUB failed to respond to boot command")
        else:
            logger.debug("SENSORHUB powered off")

        cln_motorhub_cmd = self.node_manager.node.make_client(uavcan.node.ExecuteCommand_1, self.node_manager.motorhub.id)
        motorhub_response = await cln_motorhub_cmd.call(uavcan.node.ExecuteCommand_1.Request(NodeCommands.BOOT))
        if motorhub_response is None:
            logger.error("MOTORHUB failed to respond to boot command")
        elif motorhub_response[0].status != uavcan.node.ExecuteCommand_1.Response.STATUS_SUCCESS:
            logger.error("MOTORHUB failed to respond to boot command")
        else:
            logger.debug("MOTORHUB powered off")

        cln_gps_cmd = self.node_manager.node.make_client(uavcan.node.ExecuteCommand_1, self.node_manager.gps.id)
        gps_response = await cln_gps_cmd.call(uavcan.node.ExecuteCommand_1.Request(NodeCommands.BOOT))
        if gps_response is None:
            logger.error("GPS failed to respond to boot command")
        elif gps_response[0].status != uavcan.node.ExecuteCommand_1.Response.STATUS_SUCCESS:
            logger.error("GPS failed to respond to boot command")
        else:
            logger.debug("GPS powered off")

        self.boot.set()

    @async_loop_decorator()
    async def _mainio_run_loop(self) -> None:
        try:
            await self._pub_servo_readiness.publish(self.main.txdata.servo_readiness)
            await self._pub_esc_readiness.publish(self.main.txdata.esc_readiness)

            await self._pub_elevon1_sp.publish(self.main.txdata.elevon1)
            await self._pub_elevon2_sp.publish(self.main.txdata.elevon2)
            await self._pub_tilt_sp.publish(self.main.txdata.tilt)

            await self._pub_esc1_sp.publish(self.main.txdata.esc1)
            await self._pub_esc2_sp.publish(self.main.txdata.esc2)
            await self._pub_esc3_sp.publish(self.main.txdata.esc3)
            await self._pub_esc4_sp.publish(self.main.txdata.esc4)
        except pycyphal.presentation._port._error.PortClosedError:
            pass

        await asyncio.sleep(1 / self._freq)

    #region Subscriptions
    def _on_time(self, msg: uavcan.time.SynchronizedTimestamp_1, _: pycyphal.transport.TransferFrom) -> None:
        self.main.rxdata.time.dump(msg)

    def _on_gps_time(self, msg: uavcan.time.SynchronizedTimestamp_1, _: pycyphal.transport.TransferFrom) -> None:
        # TODO: gps time transition
        if self._use_gps_time:
            self.main.rxdata.time.dump(msg)
    
    def _on_att(self, msg: reg.udral.physics.kinematics.cartesian.StateVarTs_0, _: pycyphal.transport.TransferFrom) -> None:
        self.main.rxdata.att.dump(msg)

    def _on_alt(self, msg: uavcan.si.unit.length.WideScalar_1, _: pycyphal.transport.TransferFrom) -> None:
        t = self.main.rxdata.time.time
        self.main.rxdata.alt.dump(msg, t)

    def _on_gps(self, msg: reg.udral.physics.kinematics.geodetic.PointStateVarTs_0, _: pycyphal.transport.TransferFrom) -> None:
        self.main.rxdata.gps.dump(msg, self.main.rxdata.att.yaw)

    def _on_ias(self, msg: reg.udral.physics.kinematics.translation.LinearTs_0, _: pycyphal.transport.TransferFrom) -> None:
        self.main.rxdata.ias.dump(msg)

    def _on_aoa(self, msg: uavcan.si.unit.angle.Scalar_1, _: pycyphal.transport.TransferFrom) -> None:
        t = self.main.rxdata.time.time
        self.main.rxdata.aoa.dump(msg, t)
    
    # TODO: other subscribers
    def _void_handler(self, msg, _: pycyphal.transport.TransferFrom) -> None:
        pass
    #endregion
        
    async def run(self) -> None:
        """Write to publishers."""
        self._node.heartbeat_publisher.mode = uavcan.node.Mode_1.OPERATIONAL
        logger.warning("UAVCAN Node Running...\n----- Ctrl-C to exit -----")

        await self._mainio_run_loop()

    async def close(self) -> None:
        """Close the instance."""
        logger.info("Closing UAVCANManager...") # TODO: Change to logger.debug()

        # cln_clock_cmd = self.node_manager.node.make_client(uavcan.node.ExecuteCommand_1, self.node_manager.clock.id)
        # clock_response = await cln_clock_cmd.call(uavcan.node.ExecuteCommand_1.Request(uavcan.node.ExecuteCommand_1.Request.COMMAND_POWER_OFF))
        # if clock_response is None:
        #     logger.error("CLOCK failed to respond to power off request")
        # elif clock_response[0].status != uavcan.node.ExecuteCommand_1.Response.STATUS_SUCCESS:
        #     logger.error("CLOCK failed to respond to power off request")
        # else:
        #     logger.debug("CLOCK powered off")

        # cln_sensorhub_cmd = self.node_manager.node.make_client(uavcan.node.ExecuteCommand_1, self.node_manager.sensorhub.id)
        # sensorhub_response = await cln_sensorhub_cmd.call(uavcan.node.ExecuteCommand_1.Request(uavcan.node.ExecuteCommand_1.Request.COMMAND_POWER_OFF))
        # if sensorhub_response is None:
        #     logger.error("SENSORHUB failed to respond to power off request")
        # elif sensorhub_response[0].status != uavcan.node.ExecuteCommand_1.Response.STATUS_SUCCESS:
        #     logger.error("SENSORHUB failed to respond to power off request")
        # else:
        #     logger.debug("SENSORHUB powered off")

        # cln_motorhub_cmd = self.node_manager.node.make_client(uavcan.node.ExecuteCommand_1, self.node_manager.motorhub.id)
        # motorhub_response = await cln_motorhub_cmd.call(uavcan.node.ExecuteCommand_1.Request(uavcan.node.ExecuteCommand_1.Request.COMMAND_POWER_OFF))
        # if motorhub_response is None:
        #     logger.error("MOTORHUB failed to respond to power off request")
        # elif motorhub_response[0].status != uavcan.node.ExecuteCommand_1.Response.STATUS_SUCCESS:
        #     logger.error("MOTORHUB failed to respond to power off request")
        # else:
        #     logger.debug("MOTORHUB powered off")

        # cln_gps_cmd = self.node_manager.node.make_client(uavcan.node.ExecuteCommand_1, self.node_manager.gps.id)
        # gps_response = await cln_gps_cmd.call(uavcan.node.ExecuteCommand_1.Request(uavcan.node.ExecuteCommand_1.Request.COMMAND_POWER_OFF))
        # if gps_response is None:
        #     logger.error("GPS failed to respond to power off request")
        # elif gps_response[0].status != uavcan.node.ExecuteCommand_1.Response.STATUS_SUCCESS:
        #     logger.error("GPS failed to respond to power off request")
        # else:
        #     logger.debug("GPS powered off")

        self._node.close()


class Navigator:
    """Converts navigational commands to autopilot commands."""
    class Waypoint:
        """Stores information for a waypoint."""
        count = 0
        def __init__(self, latitude: float, longitude: float, altitude: float = None, name: str | None = None) -> None:
            self.latitude = latitude
            self.longitude = longitude
            self.altitude = altitude

            Navigator.Waypoint.count += 1
            self.name = name if name is not None else f"Waypoint {Navigator.Waypoint.count}"

    def __init__(self, main: 'Main') -> None:
        """Initializes the Navigator class.

        Parameters
        ----------
        main : 'Main'
            The main object.
        """

        self.main = main
        self._waypoint_list: list[Navigator.Waypoint] = [] # Navigating from waypoint 0 to waypoint 1
        self.commanded_heading = 0.0 # DEGREES
        self.commanded_altitude = self.main.afcs._sp_altitude
        self.distance = 0.0

        self.boot = asyncio.Event()

    async def boot_proc(self) -> None:
        """Perform boot-related tasks."""
        # TODO: do boot stuff here, calibrate gps etc
        await self.main.io.node_manager.gps.active.wait()
        self.append_wpt(Navigator.Waypoint(self.main.rxdata.gps.latitude, self.main.rxdata.gps.longitude, 0.0, name='Start'))
        logger.debug(f"Navigator aligning at {self._waypoint_list[0].latitude}, {self._waypoint_list[0].longitude}")

        try:
            with open('uav/flightplan.json', 'r') as fpl_file:
                self._fpl = json.load(fpl_file)
        except FileNotFoundError:
            logger.error("Could not open flightplan.json")
            self.hover_alt = 75
            self.cruise_ias = 70 * KT_TO_MS
        else:
            waypoints = self._fpl['waypoints']
            self.append_wpt(*[Navigator.Waypoint(wpt['lat'], wpt['long'], wpt['alt_m_agl']) for wpt in waypoints])
            self.hover_alt = self._fpl['hover_alt_m_agl']
            self.cruise_ias = self._fpl['cruise_kias'] * KT_TO_MS
            logger.info(f"Navigator loaded flightplan.json with wpts: {len(waypoints)}, hover: {self.hover_alt} m, ias: {self.cruise_ias:.0f} m/s")

        self.landing_wpt = self._waypoint_list[-1]

        self.boot.set()

    def append_wpt(self, *wpts: Waypoint) -> None:
        """Add a Waypoint to the end of the flight plan."""
        for wpt in wpts:
            self._waypoint_list.append(wpt)

    def direct_wpt(self, wpt: Waypoint) -> None:
        """Direct to a waypoint."""
        self._waypoint_list.insert(1, wpt)
        self.auto_alt = True

    def next_wpt(self) -> None:
        """Increment flight plan wpt."""
        if len(self._waypoint_list)>2:
            self._waypoint_list.pop(0)
    
    def _calc_altitude(self) -> float:
        """Determine the desired altitude from the next waypoint."""
        if len(self._waypoint_list)>1:
            self.commanded_altitude = self._waypoint_list[1].altitude if self._waypoint_list[1].altitude is not None else self.main.afcs._sp_altitude
        else:
            # TODO
            pass
        return self.commanded_altitude
    
    def _detect_change(self) -> None:
        if self.distance<100 and self.main.state.custom_submode==g.CUSTOM_SUBMODE_FLIGHT_NORMAL:
            self.next_wpt()

    @async_loop_decorator(close=False)
    async def _navigator_run_loop(self) -> None:
        """Set afcs setpoints based on flight plan."""
        # TODO: navigator modes, safety checks
        hdg, _, self.distance = gps_angles(
            self.main.rxdata.gps.latitude,
            self.main.rxdata.gps.longitude,
            self.main.rxdata.alt.altitude,
            self._waypoint_list[1].latitude,
            self._waypoint_list[1].longitude,
            self._waypoint_list[1].altitude
        )
        self.commanded_heading = math.radians(hdg)
        self._calc_altitude()
        self._detect_change()
        await asyncio.sleep(0.1)

    async def run(self) -> None:
        """Calculate desired heading from flight plan."""
        logger.info("Starting Navigator")
        await self._navigator_run_loop()


class AFCS:
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
        Run the AFCS and execute control logic.
    """

    MAX_THROTTLE = 14000
    BASE_THROTTLE_PCT = 0.36

    def __init__(self, main: 'Main') -> None:
        """Initialize the AFCS class.

        Parameters
        ----------
        main : 'Main'
            The main object.
        """

        self.main = main
        self.boot = asyncio.Event()

        self._vpath = 0.0
        self._dyaw = 0.0
        self._ias_scalar = 1.0
        self._vtol_ratio = 1.0

        self._servos: np.ndarray = np.zeros(3, dtype=np.float16)
        self._throttles: np.ndarray = np.zeros(4, dtype=np.float16)

        self._fservos = np.zeros(3, dtype=np.float16) # elevons*2, rudder, wingtilt
        self._vservos = np.zeros(3, dtype=np.float16)

        self._fthrottles = np.zeros(4, dtype=np.float16) # throttles*4
        self._vthrottles = np.zeros(4, dtype=np.float16)

        self.auto_alt = True
        self.auto_ias = True

        self._spf_ias = 40.0
        self._sp_altitude = 100.0
        self._sp_heading = 0.0

        self._spf_vpath = 0.0
        self._spf_aoa = 0.1
        self._spf_roll = 0.0
        self._spf_rollspeed = 0.0

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
        self._outv_thr_mode = 0 # 0-settle 1-hover 2-launch

        self._ftilt = math.pi/2
        self._rtilt = math.pi/2
        self._last_tilt = 0.0
        self._outt_pitch = 0.0

        # self._pid{f or v}_{from}_{to}

        self._pidf_alt_vpa = PID(kp=0.007, ti=0.006, td=0.1, integral_limit=0.05, maximum=0.1, minimum=-0.15)
        self._pidf_vpa_aoa = PID(kp=0.7, ti=3.0, td=0.05, integral_limit=0.2, maximum=math.pi/6, minimum=0.02)
        self._pidf_vpa_thr = PID(kp=0.35, ti=0.003, td=0.0, integral_limit=0.25, maximum=0.25, minimum=0.0)
        self._pidf_aoa_out = PID(kp=-0.10, ti=-0.008, td=0.02, integral_limit=2.5, maximum=0.2, minimum=-0.2)
        self._pidf_dyw_rol = PID(kp=-0.75, ti=-8.0, td=0.002, integral_limit=0.1, maximum=math.pi/6, minimum=-math.pi/6)
        self._pidf_rol_rls = PID(kp=1.5, ti=6.0, td=0.02, integral_limit=0.2, maximum=2.0, minimum=-2.0)
        self._pidf_rls_out = PID(kp=0.005, ti=0.003, td=0.005, integral_limit=0.1, maximum=0.1, minimum=-0.1)
        self._pidf_ias_thr = PID(kp=0.08, ti=4.0, td=0.1, integral_limit=1.0, maximum=(1-AFCS.BASE_THROTTLE_PCT), minimum=0.0) # TODO

        self._pidv_xdp_xsp = PID(kp=0.0, ti=0.0, td=0.0, integral_limit=None, minimum=-5.0, maximum=5.0)
        self._pidv_xsp_rol = PID(kp=0.1, ti=0.9, td=0.1, integral_limit=0.3, minimum=-math.pi/12, maximum=math.pi/12) # TODO
        self._pidv_rol_rls = PID(kp=1.0, ti=1.0, td=0.05, integral_limit=0.08, minimum=-math.pi/6, maximum=math.pi/6)
        self._pidv_rls_out = PID(kp=0.021, ti=0.05, td=0.04, integral_limit=0.3, minimum=-0.08, maximum=0.08)

        self._pidv_ydp_ysp = PID(kp=0.0, ti=0.0, td=0.0, integral_limit=None, minimum=-5.0, maximum=5.0)
        self._pidv_ysp_pit = PID(kp=-0.35, ti=-1.5, td=0.0, integral_limit=0.3, minimum=-math.pi/8, maximum=math.pi/12)
        self._pidv_pit_pts = PID(kp=0.7, ti=0.8, td=0.0, integral_limit=0.1, minimum=-math.pi/6, maximum=math.pi/6)
        self._pidv_pts_out = PID(kp=0.027, ti=0.03, td=0.06, integral_limit=1.0, minimum=-0.1, maximum=0.1)

        self._pidv_alt_vsp = PID(kp=0.5, ti=1.0, td=0.05, integral_limit=0.5, minimum=-1.5, maximum=2.0)
        self._pidv_vsp_out = PID(kp=0.18, ti=0.4, td=0.001, integral_limit=5.0, minimum=0.0, maximum=0.68)

        self._pidv_dyw_yws = PID(kp=-0.5, ti=1.0, td=0.0, integral_limit=0.2, minimum=-math.pi/6, maximum=math.pi/6)
        self._pidv_yws_out = PID(kp=0.2, ti=0.3, td=0.01, integral_limit=0.15, minimum=-math.pi/24, maximum=math.pi/24)

        self._pidt_dep_out = PID(kp=0.4, ti=0.0, td=0.0, integral_limit=None, minimum=None, maximum=None)
        self._pidt_arr_out = PID(kp=0.5, ti=0.0, td=0.0, integral_limit=None, minimum=None, maximum=None)
        # TODO: arrival needs to bleed off energy first

    async def boot_proc(self) -> None:
        """Perform boot-related tasks."""
        await self.main.navigator.boot.wait()

    #region Calculations
    def _flight_calc(self, wipe: bool = True) -> np.ndarray:
        """Calculate flight servo commands from sensors."""
        if (alt:=self.main.rxdata.alt).dt > 0.0:
            self._spf_vpath = self._pidf_alt_vpa.cycle(alt.altitude, self._sp_altitude, alt.dt)
            self.main.rxdata.alt.dt = 0.0 if wipe else alt.dt

        if any([(att:=self.main.rxdata.att).dt > 0.0, (aoa:=self.main.rxdata.aoa).dt > 0.0]):
            if not aoa.dt > 0.0:
                dt = att.dt
            elif not att.dt > 0.0:
                dt = aoa.dt
            else:
                dt = (att.dt + aoa.dt) / 2

            self._vpath = att.pitch - math.cos(att.roll) * aoa.aoa
            self._spf_aoa = self._pidf_vpa_aoa.cycle(self._vpath, self._spf_vpath, dt)
            self._throttle_vpa_corr = self._pidf_vpa_thr.cycle(self._vpath, self._spf_vpath, dt)

        if (att:=self.main.rxdata.att).dt > 0.0:
            self._dyaw = calc_dyaw(att.yaw, self._sp_heading)

            self._spf_roll = self._pidf_dyw_rol.cycle(self._dyaw, 0.0, att.dt)
            self._spf_rollspeed = self._pidf_rol_rls.cycle(att.roll, self._spf_roll, att.dt)
            self._outf_roll = self._pidf_rls_out.cycle(att.rollspeed, self._spf_rollspeed, att.dt)

            self._throttle_roll_corr = abs(math.sin(att.roll)) if abs(att.roll)>math.pi/24 else 0.0

            self.main.rxdata.att.dt = 0.0 if wipe else att.dt

        if (aoa:=self.main.rxdata.aoa).dt > 0.0:
            self._outf_pitch = self._pidf_aoa_out.cycle(aoa.aoa, self._spf_aoa, aoa.dt)
            self.main.rxdata.aoa.dt = 0.0 if wipe else aoa.dt

        if (ias:=self.main.rxdata.ias).dt > 0.0:
            self._outf_throttle_ias = AFCS.BASE_THROTTLE_PCT+(1+3*self._throttle_roll_corr)*self._pidf_ias_thr.cycle(ias.ias, self._spf_ias, ias.dt)
            self.main.rxdata.ias.dt = 0.0 if wipe else ias.dt

        self._fservos[0] = self._outf_pitch + self._outf_roll
        self._fservos[1] = self._outf_pitch - self._outf_roll
        self._fservos[2] = 0.0
        self._fservos = self._fservos.clip(-10.0, 10.0) # overflow protection (not servo limits)

        self._throttle_roll_corr = min(self._throttle_roll_corr, 1.0)
        self._throttle_vpa_corr = min(self._throttle_vpa_corr, 1.0)

        self._fthrottles.fill(self._outf_throttle_ias + 0*self._throttle_vpa_corr)
        self._fthrottles = self._fthrottles.clip(0.0, 1.0)
        self._fthrottles *= AFCS.MAX_THROTTLE

        if wipe:
            self._outt_pitch = 0.0
            self._ftilt = 0.0
            self._rtilt = 0.0

        return self._fservos, self._fthrottles

    def _vtol_calc(self) -> np.ndarray:
        """Calculate VTOL throttle commands from sensors."""
        if (alt:=self.main.rxdata.alt).dt > 0.0:
            self._spv_vs = self._pidv_alt_vsp.cycle(alt.altitude, self._sp_altitude, alt.dt)
            self._outv_thr_mode = 1
            if alt.altitude < 0.5:
                if self.main.state.custom_submode == g.CUSTOM_SUBMODE_TAKEOFF_ASCENT:
                    self._outv_thr_mode = 2
                elif alt.altitude < 0.1:
                    self._outv_thr_mode = 0

            if alt.altitude < 0.05:
                self._pidv_pts_out._integral = 0.05
                self._pidv_xdp_xsp._integral = 0.0
                self._pidv_xsp_rol._integral = 0.0
                self._pidv_rol_rls._integral = 0.0
                self._pidv_rls_out._integral = 0.0
                self._pidv_ydp_ysp._integral = 0.0
                self._pidv_ysp_pit._integral = 0.0
                self._pidv_pit_pts._integral = 0.0
                self._pidv_alt_vsp._integral = 0.0
                self._pidv_vsp_out._integral = 0.0
                self._pidv_dyw_yws._integral = 0.0
                self._pidv_yws_out._integral = 0.0
                self._pidv_vsp_out.minimum = 0.0
            else:
                self._pidv_vsp_out.minimum = 0.50
            self.main.rxdata.alt.dt = 0.0

        if (cam:=self.main.rxdata.cam).dt > 0.0:
            # self._spv_xspeed = self._pidv_xdp_xsp.cycle(cam.xdp, 0.0, cam.dt)
            # self._spv_yspeed = self._pidv_ydp_ysp.cycle(cam.ydp, 0.0, cam.dt)
            self.main.rxdata.cam.dt = 0.0

        if (gps:=self.main.rxdata.gps).dt > 0.0:
            self.main.rxdata.gps.dt = 0.0

        if (att:=self.main.rxdata.att).dt > 0.0:
            if self.main.state.custom_submode in [g.CUSTOM_SUBMODE_TAKEOFF_TRANSIT, g.CUSTOM_SUBMODE_TAKEOFF_DEPART]:
                self._spv_yspeed = 30

                if self.main.state.custom_submode == g.CUSTOM_SUBMODE_TAKEOFF_TRANSIT:
                    self._last_tilt = self._outt_pitch
                    self._outt_pitch = self._pidt_dep_out.cycle(att.pitchspeed, self._spv_pitchspeed, att.dt) * att.dt * 1e-6
                    # base = 3 * att.dt * 1e-7
                    # self._rtilt-=base
                    # self._ftilt-=1.2*base
                    if self._outt_pitch>0.0: # put the nose up (lower back wing a little)!
                        self._rtilt -= self._outt_pitch#-self._last_tilt
                        self._rtilt = max(min(self._rtilt, math.pi/2), 0.0)
                    else: # put the nose down (lower front wing a little)!
                        self._ftilt += self._outt_pitch
                        self._ftilt = max(min(self._ftilt, math.pi/2), 0.0)
                else:
                    self._outt_pitch = 0.0
                    self._ftilt = math.pi/2
                    self._rtilt = math.pi/2
            elif self.main.state.custom_submode == g.CUSTOM_SUBMODE_LANDING_TRANSIT:
                self._spv_yspeed = 0

                self._last_tilt = self._outt_pitch
                self._outt_pitch = self._pidt_arr_out.cycle(att.pitch, 0.0, att.dt)
                if self._outt_pitch>self._last_tilt: # put the nose up (raise front wing a little)!
                    self._ftilt += self._outt_pitch-self._last_tilt
                    self._ftilt = max(min(self._ftilt, math.pi/2), 0.0)
                else: # put the nose down (raise back wing a little)!
                    self._rtilt += self._last_tilt-self._outt_pitch
                    self._rtilt = max(min(self._rtilt, math.pi/2), 0.0)
            else:
                self._outt_pitch = 0.0
                self._ftilt = 0.0
                self._rtilt = 0.0
    
            self._outv_throttle = self._pidv_vsp_out.cycle(att.zspeed, self._spv_vs, att.dt)
            self._spv_roll = self._pidv_xsp_rol.cycle(att.xspeed, self._spv_xspeed, att.dt)
            self._spv_pitch = self._pidv_ysp_pit.cycle(att.yspeed, self._spv_yspeed, att.dt)
            if self.main.state.custom_submode == g.CUSTOM_SUBMODE_TAKEOFF_TRANSIT:
                self._spv_pitch = -0.5
            self._spv_rollspeed = self._pidv_rol_rls.cycle(att.roll, self._spv_roll, att.dt)
            self._spv_pitchspeed = self._pidv_pit_pts.cycle(att.pitch, self._spv_pitch, att.dt)
            self._outv_roll = self._pidv_rls_out.cycle(att.rollspeed, self._spv_rollspeed, att.dt)
            self._outv_pitch = self._pidv_pts_out.cycle(att.pitchspeed, self._spv_pitchspeed, att.dt)

            self._dyaw = calc_dyaw(att.yaw, self._sp_heading) if self.main.rxdata.alt.altitude>3 else 0.0
            self._spv_yawspeed = self._pidv_dyw_yws.cycle(self._dyaw, 0.0, att.dt)
            self._outv_yaw = self._pidv_yws_out.cycle(att.yawspeed, self._spv_yawspeed, att.dt)
            self.main.rxdata.att.dt = 0.0

        match self._outv_thr_mode:
            case 0:
                self._outv_throttle = 0.0
                self.main.state.set_mode(m.MAV_MODE_GUIDED_ARMED, g.CUSTOM_MODE_GROUND, g.CUSTOM_SUBMODE_GROUND_DISARMED)
            case 1:
                pass
            case 2:
                self._outv_throttle = 0.58
                self._pidv_vsp_out._integral = 3.2

        t1 = self._outv_pitch + self._outv_throttle
        t2 = self._outv_pitch + self._outv_throttle
        t3 = -self._outv_pitch + self._outv_roll + self._outv_throttle
        t4 = -self._outv_pitch - self._outv_roll + self._outv_throttle
        self._vthrottles = np.array([t1, t2, t3, t4], dtype=np.float16)
        self._vthrottles *= AFCS.MAX_THROTTLE

        self._vservos[0] = math.pi/2 - self._outv_yaw
        self._vservos[1] = math.pi/2 + self._outv_yaw
        self._vservos[2] = math.pi/2

        return self._vservos, self._vthrottles
    #endregion

    @async_loop_decorator()
    async def _afcs_run_loop(self) -> None:
        self._vtol_ratio = 1 - (self.main.rxdata.ias.ias/20) # 1 is VTOL
        try:
            self._ias_scalar = 676 / (self.main.rxdata.ias.ias**2) # ~30**2 / ias**2
        except ZeroDivisionError:
            self._ias_scalar = 1.0

        self._vtol_ratio = max(min(self._vtol_ratio, 1.0), 0.0)
        self._ias_scalar = min(self._ias_scalar, 10.0)

        # Setpoints
        match self.main.state.custom_submode:
            case g.CUSTOM_SUBMODE_LANDING_DESCENT:
                self._sp_altitude = 0.0
                self._sp_heading = self.main.navigator.commanded_heading
            case g.CUSTOM_SUBMODE_TAKEOFF_ASCENT | g.CUSTOM_SUBMODE_TAKEOFF_DEPART | g.CUSTOM_SUBMODE_TAKEOFF_TRANSIT | g.CUSTOM_SUBMODE_LANDING_TRANSIT | g.CUSTOM_SUBMODE_LANDING_HOVER:
                self._sp_altitude = self.main.navigator.hover_alt
                self._spf_ias = self.main.navigator.cruise_ias

                self._sp_heading = self.main.navigator.commanded_heading
            case g.CUSTOM_SUBMODE_FLIGHT_NORMAL:
                if self.auto_alt:
                    self._sp_altitude = self.main.navigator.commanded_altitude
                if self.auto_ias:
                    self._spf_ias = self.main.navigator.cruise_ias

                self._sp_heading = self.main.navigator.commanded_heading
            case _:
                self._sp_altitude = 0.0

        # Controls
        match self.main.state.custom_submode:
            case g.CUSTOM_SUBMODE_TAKEOFF_ASCENT | g.CUSTOM_SUBMODE_TAKEOFF_DEPART | g.CUSTOM_SUBMODE_LANDING_DESCENT | g.CUSTOM_SUBMODE_LANDING_HOVER:
                # VTOL
                self._servos, self._throttles = self._vtol_calc()
            case g.CUSTOM_SUBMODE_TAKEOFF_TRANSIT:
                # Departure transition
                fservos, fthrottles = self._flight_calc(wipe=False)
                _, vthrottles = self._vtol_calc()
                self._servos[2] = self._ftilt
                self._servos[:2] = (1-self._vtol_ratio)*fservos[:2] + self._rtilt
                self._throttles = (1-self._vtol_ratio)*fthrottles + self._vtol_ratio*vthrottles
            case g.CUSTOM_SUBMODE_LANDING_TRANSIT:
                # Arrival transition
                fservos, fthrottles = self._flight_calc(wipe=False)
                _, vthrottles = self._vtol_calc()
                self._servos[2] = self._ftilt
                self._servos[:2] = (1-self._vtol_ratio)*fservos[:2] + self._rtilt
                self._throttles = (1-self._vtol_ratio)*fthrottles + self._vtol_ratio*vthrottles
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
                self._servos = np.array([math.pi/2, math.pi/2, math.pi/2, math.pi/2], dtype=np.float16)
                self._throttles = np.zeros(4, dtype=np.float16)

        # Mode increments
        match self.main.state.custom_submode:
            case g.CUSTOM_SUBMODE_TAKEOFF_ASCENT:
                if self.main.rxdata.alt.altitude > self.main.navigator.hover_alt-3:
                    self.main.state.inc_mode()
            case g.CUSTOM_SUBMODE_TAKEOFF_DEPART:
                if self.main.rxdata.ias.ias > 3:
                    self.main.state.inc_mode()
            case g.CUSTOM_SUBMODE_TAKEOFF_TRANSIT:
                if self._vtol_ratio==0 and self._ftilt==0 and self._rtilt==0:
                    self.main.state.inc_mode()
                    self._pidf_ias_thr.reset()
            case g.CUSTOM_SUBMODE_LANDING_TRANSIT:
                # TODO
                if False:
                    self.main.state.inc_mode()
            case g.CUSTOM_SUBMODE_LANDING_HOVER:
                # TODO
                if False:
                    self.main.state.inc_mode()
            case g.CUSTOM_SUBMODE_LANDING_DESCENT:
                # TODO
                if False:
                    self.main.state.inc_mode()
            case g.CUSTOM_SUBMODE_FLIGHT_NORMAL | g.CUSTOM_SUBMODE_FLIGHT_TERRAIN_AVOIDANCE:
                pass # TODO: terr avoidance check

        self._servos[:2] = np.clip(self._servos[:2], -math.pi/12, 7*math.pi/12)
        self._servos[2] = np.clip(self._servos[2], 0.0, math.pi/2)
        self._throttles = np.clip(self._throttles, 0, 14000)
        self._servos = np.nan_to_num(self._servos)
        self._throttles = np.nan_to_num(self._throttles)

        self.main.txdata.elevon1 = self._servos[0]
        self.main.txdata.elevon2 = self._servos[1]
        self.main.txdata.tilt = self._servos[2]
        
        self.main.txdata.esc1 = self._throttles[0]
        self.main.txdata.esc2 = self._throttles[1]
        self.main.txdata.esc3 = self._throttles[2]
        self.main.txdata.esc4 = self._throttles[3]

        await asyncio.sleep(0)

    async def run(self) -> None:
        """Calculate desired control positions."""
        logger.info("Starting AFCS")
        await self._afcs_run_loop()

    async def close(self) -> None:
        logger.info("Closing AFCS")
        pass


class CommManager:
    """CommManager class for managing MAVLINK communication with GCS.

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
        """Initialize a CommManager instance.

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

        logging.addLevelName(MAVLOG_DEBUG, 'MAVdebug')
        logging.addLevelName(MAVLOG_TX, 'TX')
        logging.addLevelName(MAVLOG_RX, 'RX')
        logging.addLevelName(MAVLOG_LOG, 'LOG')

        _formatter = logging.Formatter(str(os.getpid()) + ' (%(asctime)s - %(name)s - %(levelname)s) %(message)s')
        _filehandler = logging.FileHandler('mavlog.log', mode='a')
        _filehandler.setFormatter(_formatter)

        self._mavlogger = logging.getLogger(f'UAV{self.main.systemid}')
        self._mavlogger.addHandler(_filehandler)
        self._mavlogger.addHandler(streamhandler)
        self._mavlogger.setLevel(MAVLOG_TX)

        self._mavlogger.log(MAVLOG_TX, "Test TX")
        self._mavlogger.log(MAVLOG_RX, "Test RX")
        self._mavlogger.log(MAVLOG_LOG, "Test Log")

        self._gcs_id = None
        self._mav_conn_gcs: mavutil.mavfile = mavutil.mavlink_connection(self.main.config.get('mavlink', 'uav_gcs_conn'), source_system=self.main.systemid, source_component=m.MAV_COMP_ID_AUTOPILOT1, input=False, autoreconnect=True)
        import common.key as key
        self._mav_conn_gcs.setup_signing(key.KEY.encode('utf-8'))
        self._cam_id = self.main.config.getint('mavlink_ids', 'cam_id')

    async def boot_proc(self):
        import common.key as key
        await self._establish_cam(key=key.CAMKEY.encode('utf-8'))

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
            for i in range(CommManager.MAX_FLUSH_BUFFER):
                msg = connection.recv_match(blocking=False)
                if msg is None:
                    logger.debug(f"Buffer flushed, {i} messages cleared")
                    break
                elif i >= CommManager.MAX_FLUSH_BUFFER-1:
                    logger.error(f"Messages still in buffer after {CommManager.MAX_FLUSH_BUFFER} flush cycles")     
        except (ConnectionError, OSError):
            logger.debug("No connection to flush.")

    async def _establish_cam(self, key: bytes, timeout_cycles: int = 5):
        try:
            self._cam_conn: mavutil.mavfile = mavutil.mavlink_connection(self.main.config.get('mavlink', 'uav_camera_conn'), source_system=self.main.systemid, source_component=m.MAV_COMP_ID_AUTOPILOT1, input=True)
            self._cam_conn.setup_signing(key)

            CommManager.flush_buffer(self._cam_conn)

            self._mavlogger.log(MAVLOG_TX, f"Connecting to camera #{self._cam_id}...")

            self._cam_conn.mav.change_operator_control_send(self._cam_id, 0, 0, key)
            await asyncio.sleep(1)

            while not self.main.stop.is_set():
                try:
                    msg = self._cam_conn.recv_msg()
                except ConnectionResetError:
                    raise CommManager.PreExistingConnection

                if msg is not None and msg.get_type()=='CHANGE_OPERATOR_CONTROL_ACK':# and msg.get_srcSystem()==target and msg.gcs_system_id==self.main.systemid:
                    match msg.ack:
                        case 0:
                            self._mavlogger.log(MAVLOG_LOG, f"Connected to camera #{self._cam_id}")
                            break
                        case 1 | 2:
                            self._mavlogger.log(MAVLOG_RX, f"Bad connection key for camera #{self._cam_id}")
                            break
                        case 3:
                            self._mavlogger.log(MAVLOG_RX, f"Camera #{self._cam_id} is connected to another UAV")
                            break

                self._cam_conn.mav.change_operator_control_send(self._cam_id, 0, 0, key)
                await asyncio.sleep(0)
        except KeyboardInterrupt:
            pass

    #region Handlers
    def _handle_heartbeat(self, msg) -> None:
        """Handle HEARTBEAT messages."""
        self._mavlogger.log(MAVLOG_DEBUG, f"Heartbeat message from link #{msg.get_srcSystem()}")

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
                        self._mavlogger.log(MAVLOG_LOG, f"Accepting control request from GCS ({msg.get_srcSystem()})")
                    self._gcs_id = msg.get_srcSystem()
                    self._mav_conn_gcs.mav.change_operator_control_ack_send(msg.get_srcSystem(), msg.control_request, 0)
                else:
                    # Already controlled
                    self._mavlogger.log(MAVLOG_RX, f"Rejecting second control request from {msg.get_srcSystem()}")
                    self._mav_conn_gcs.mav.change_operator_control_ack_send(msg.get_srcSystem(), msg.control_request, 3)
            elif msg.control_request == 1 and msg.passkey==key.KEY:# and self._gcs_id is not None:
                # Accepted (released)
                self._mavlogger.log(MAVLOG_LOG, f"Releasing from GCS ({self._gcs_id})")
                self._gcs_id = None
                self._mav_conn_gcs.mav.change_operator_control_ack_send(msg.get_srcSystem(), msg.control_request, 0)
            else:
                # Bad key
                self._mavlogger.log(MAVLOG_RX, f"Bad key in GCS control request")
                self._mav_conn_gcs.mav.change_operator_control_ack_send(msg.get_srcSystem(), msg.control_request, 1)

    def _handle_command_long(self, msg) -> None:
        """Handle COMMAND_LONG messages."""
        if msg.target_system==self.main.systemid and msg.get_srcSystem()==self._gcs_id:
            match msg.command:
                # DO_SET_MODE
                case m.MAV_CMD_DO_SET_MODE:
                    self._mavlogger.log(MAVLOG_RX, "Mode change requested")
                        
                    if self.main.state.set_mode(msg.param1, msg.param2, msg.param3):
                        self._mav_conn_gcs.mav.command_ack_send(m.MAV_CMD_DO_SET_MODE, m.MAV_RESULT_ACCEPTED, 255, 0, 0, 0)
                    else:
                        self._mav_conn_gcs.mav.command_ack_send(m.MAV_CMD_DO_SET_MODE, m.MAV_RESULT_DENIED, 255, 0, 0, 0)
                # DO_CHANGE_ALTITUDE
                case m.MAV_CMD_DO_CHANGE_ALTITUDE:
                    try:
                        if msg.param1 >= 0:
                            self.main.afcs.auto_alt = False
                            self.main.afcs._sp_altitude = msg.param1 # TODO add checks!
                            self._mavlogger.log(MAVLOG_RX, f"GCS commanded altitude setpoint to {msg.param1}")
                        else:
                            self.main.afcs.auto_alt = True
                            self._mavlogger.log(MAVLOG_RX, f"GCS commanded altitude setpoint to flight plan")
                        self._mav_conn_gcs.mav.command_ack_send(m.MAV_CMD_DO_CHANGE_ALTITUDE, m.MAV_RESULT_ACCEPTED, 255, 0, 0, 0)
                    except AttributeError:
                        self._mav_conn_gcs.mav.command_ack_send(m.MAV_CMD_DO_CHANGE_ALTITUDE, m.MAV_RESULT_TEMPORARILY_REJECTED, 255, 0, 0, 0)
                # DO_CHANGE_SPEED
                case m.MAV_CMD_DO_CHANGE_SPEED:
                    try:
                        if msg.param2 >= 0:
                            self.main.afcs.auto_ias = False
                            self.main.afcs._spf_ias = msg.param2 # TODO add checks!
                            self._mavlogger.log(MAVLOG_RX, f"GCS commanded speed setpoint to {msg.param2}")
                        else:
                            self.main.afcs.auto_ias = True
                            self._mavlogger.log(MAVLOG_RX, f"GCS commanded speed setpoint to flight plan")
                        self._mav_conn_gcs.mav.command_ack_send(m.MAV_CMD_DO_CHANGE_SPEED, m.MAV_RESULT_ACCEPTED, 255, 0, 0, 0)
                    except AttributeError:
                        self._mav_conn_gcs.mav.command_ack_send(m.MAV_CMD_DO_CHANGE_SPEED, m.MAV_RESULT_TEMPORARILY_REJECTED, 255, 0, 0, 0)
                # DO_GIMBAL_MANAGER_PITCHYAW
                case m.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW:
                    if self.main.state.custom_submode not in [g.CUSTOM_SUBMODE_TAKEOFF_ASCENT, g.CUSTOM_SUBMODE_LANDING_HOVER, g.CUSTOM_SUBMODE_LANDING_DESCENT]:
                        self._mavlogger.log(MAVLOG_RX, f"GCS commanding camera to pitch:{msg.param1}, yaw: {msg.param2}")
                        self._cam_conn.mav.gimbal_device_set_attitude_send(
                            self._cam_id,
                            m.MAV_COMP_ID_CAMERA,
                            m.GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME,
                            euler_to_quaternion(0.0, math.radians(msg.param1), math.radians(msg.param2)),
                            0.0, 0.0, 0.0 # angular velocities
                        )
                        self._cam_conn.mav.command_ack_send(m.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW, m.MAV_RESULT_ACCEPTED, 255, 0, 0, 0)
                    else:
                        self._cam_conn.mav.command_ack_send(m.MAV_CMD_DO_SET_ROI_LOCATION, m.MAV_RESULT_TEMPORARILY_REJECTED, 255, 0, 0, 0)
                # DO_GIMBAL_SET_ROI_NONE
                case m.MAV_CMD_DO_SET_ROI_NONE:
                    if self.main.state.custom_submode not in [g.CUSTOM_SUBMODE_TAKEOFF_ASCENT, g.CUSTOM_SUBMODE_LANDING_HOVER, g.CUSTOM_SUBMODE_LANDING_DESCENT]:
                        self._mavlogger.log(MAVLOG_RX, f"GCS commanding camera to reset ROI")
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
                    else:
                        self._cam_conn.mav.command_ack_send(m.MAV_CMD_DO_SET_ROI_LOCATION, m.MAV_RESULT_TEMPORARILY_REJECTED, 255, 0, 0, 0)
                # Command unsupported
                case _:
                    self._mav_conn_gcs.mav.command_ack_send(msg.command, m.MAV_RESULT_UNSUPPORTED, 255, 0, 0, 0)

    def _handle_command_int(self, msg) -> None:
        """Handle COMMAND_INT messages."""
        if msg.target_system==self.main.systemid and msg.get_srcSystem()==self._gcs_id:
            match msg.command:
                # DO_REPOSITION
                case m.MAV_CMD_DO_REPOSITION:
                    self.main.navigator.direct_wpt(
                        Navigator.Waypoint(msg.x / 1e7, msg.y / 1e7, msg.z) if msg.z > 0.1 else Navigator.Waypoint(msg.x / 1e7, msg.y / 1e7)
                    )

                    self._mavlogger.log(MAVLOG_LOG, f"Directing to {self.main.navigator._waypoint_list[1].latitude}, {self.main.navigator._waypoint_list[1].longitude}")
                # TEMPORARY PID
                case 0:
                    # PID TUNER GOTO
                    if msg.x==0:
                        # Wildcard
                        self.main.afcs._pidf_ias_thr.set(kp=msg.param1, ti=msg.param2, td=msg.param3)
                        self.main.afcs._pidf_ias_thr.reset()
                        # self.main.afcs._spv_vs=msg.param4
                    else:
                        from utilities import pid_tune_map_names as p, pid_tune_map_sps as s
                        getattr(self.main.afcs, '_'+p[msg.x]).set(kp=msg.param1, ti=msg.param2, td=msg.param3)
                        logger.info(f"Setting PID {'_'+p[msg.x]}")
                        if s[msg.x]:
                            setattr(self.main.afcs, s[msg.x], msg.param4)
                            logger.info(f"Setting SP {s[msg.x]}")
                    logger.info(f"PID state: {msg.param1:.4f}, {msg.param2:.4f}, {msg.param3:.4f} @ {msg.param4:.4f}")
                # IMAGE
                case m.MAV_CMD_IMAGE_START_CAPTURE:
                    logger.info("Commanding camera...")
                    try:
                        self._cam_conn.mav.command_long_send(
                            self._cam_id,
                            m.MAV_COMP_ID_CAMERA,
                            m.MAV_CMD_IMAGE_START_CAPTURE,
                            0,
                            msg.param1, # param1: Camera ID
                            msg.param2, # param2: Interval (seconds)
                            msg.param3, # param3: Number images
                            msg.param4, # param4: Single image sequence number
                            0,
                            0,
                            float('nan')
                        )
                        self._mav_conn_gcs.mav.command_ack_send(m.MAV_CMD_IMAGE_START_CAPTURE, m.MAV_RESULT_ACCEPTED, 255, 0, 0, 0)
                    except AttributeError:
                        self._mav_conn_gcs.mav.command_ack_send(m.MAV_CMD_IMAGE_START_CAPTURE, m.MAV_RESULT_DENIED, 255, 0, 0, 0)
                # NAV_VTOL_TAKEOFF
                case m.MAV_CMD_NAV_VTOL_TAKEOFF:
                    if self.main.state.set_mode(m.MAV_MODE_GUIDED_ARMED, g.CUSTOM_MODE_TAKEOFF, g.CUSTOM_SUBMODE_TAKEOFF_ASCENT):
                        self._mav_conn_gcs.mav.command_ack_send(m.MAV_CMD_NAV_VTOL_TAKEOFF, m.MAV_RESULT_ACCEPTED, 255, 0, 0, 0)
                    else:
                        self._mav_conn_gcs.mav.command_ack_send(m.MAV_CMD_NAV_VTOL_TAKEOFF, m.MAV_RESULT_DENIED, 255, 0, 0, 0)
                # NAV_LAND
                case m.MAV_CMD_NAV_LAND:
                    pass # TODO
                # NAV_VTOL_LAND
                case m.MAV_CMD_NAV_VTOL_LAND:
                    pass # TODO
                # DO_GIMBAL_SET_ROI_LOCATION
                case m.MAV_CMD_DO_SET_ROI_LOCATION:
                    if self.main.state.custom_submode not in [g.CUSTOM_SUBMODE_TAKEOFF_ASCENT, g.CUSTOM_SUBMODE_LANDING_HOVER, g.CUSTOM_SUBMODE_LANDING_DESCENT]:
                        lat = msg.x / 1e7
                        lon = msg.y / 1e7
                        alt = msg.z
                        self._mavlogger.log(MAVLOG_RX, f"GCS commanding camera to lat:{lat}, lon: {lon}, alt: {alt}")
                        self._roi = [lat, lon, alt]
                        if self._roi_task is None:
                            self._roi_task = asyncio.create_task(self._roi_calc())
                        self._cam_conn.mav.command_ack_send(m.MAV_CMD_DO_SET_ROI_LOCATION, m.MAV_RESULT_ACCEPTED, 255, 0, 0, 0)
                    else:
                        self._cam_conn.mav.command_ack_send(m.MAV_CMD_DO_SET_ROI_LOCATION, m.MAV_RESULT_TEMPORARILY_REJECTED, 255, 0, 0, 0)
                # Command unsupported
                case _:
                    self._mav_conn_gcs.mav.command_ack_send(msg.command, m.MAV_RESULT_UNSUPPORTED, 255, 0, 0, 0)
    #endregion
    
    @async_loop_decorator()
    async def _manager_loop(self) -> None:
        await asyncio.sleep(0)

    async def manager(self) -> None:
        """Manage the comm's various operations."""
        asyncio.create_task(self._heartbeat())
        asyncio.create_task(self._rx())
        asyncio.create_task(self._rxcam())
        asyncio.create_task(self._tx())

        logger.info("Starting CommManager")

        await self._manager_loop()

    @async_loop_decorator(close=False)
    async def _comm_rx_loop(self) -> None:
        """Recieve messages from GCS over mavlink."""
        try:
            msg = self._mav_conn_gcs.recv_msg()
        except (ConnectionError, OSError):
            try:
                self._mavlogger.log(MAVLOG_DEBUG, "CommManager (rx) connection refused")
                await asyncio.sleep(0)
            except asyncio.exceptions.CancelledError:
                self.main.stop.set()
                raise
            finally:
                return

        if msg is not None:
            type_ = msg.get_type()
            if type_ in CommManager.type_handlers:
                handler = getattr(self, CommManager.type_handlers[type_])
                handler(msg)
            else:
                self._mavlogger.log(MAVLOG_RX, f"Unknown message type: {type_}")

        await asyncio.sleep(0)

    @async_loop_decorator(close=False)
    async def _comm_rxcam_loop(self) -> None:
        """Recieve messages from GCS over mavlink."""
        try:
            msg = self._cam_conn.recv_msg()
        except (ConnectionError, OSError, AttributeError):
            self._mavlogger.log(MAVLOG_DEBUG, "No connection to listen to.")
            return

        target = self.main.config.getint('mavlink_ids', 'cam_id')

        if msg is not None:
            if msg.get_type() == 'HEARTBEAT' and msg.get_srcSystem()==target:
                self._mavlogger.log(MAVLOG_DEBUG, f"Heartbeat message from camera #{msg.get_srcSystem()}")
                self._last_cam_beat = self.main.rxdata.time.time
            elif msg.get_type() == 'CAMERA_IMAGE_CAPTURED' and msg.get_srcSystem()==self._cam_id:
                return
                logger.info(f"Proccesing image {msg.file_url}")

                if out := await asyncio.to_thread(img.sync_proc, msg.file_url):
                    dx, dy, confidence, image = out
                    logger.info(f"'H' detected in {msg.file_url} at ({dx},{dy}) with a confidence of {confidence:.2f}.")
                    self.main.rxdata.cam.dump(dx, dy, self.main.rxdata.time.time)
                    # await grapher.imshow(image)
                else:
                    self.main.rxdata.cam.dump(0.0, 0.0, self.main.rxdata.time.time)
                    logger.info(f"None detected in {msg.file_url}.")

                self._mav_conn_gcs.mav.camera_image_captured_send(
                    int(self.main.rxdata.time.time*1e-3),
                    0, # UTC
                    0, # depr
                    int(self.main.rxdata.gps.latitude*1e7), # lat 1e7
                    int(self.main.rxdata.gps.longitude*1e7), # lon 1e7
                    int(self.main.rxdata.gps.altitude*1e3), # alt mm
                    int(self.main.rxdata.alt.altitude*1e3), # alt mm
                    euler_to_quaternion(
                        self.main.rxdata.att.roll,
                        self.main.rxdata.att.pitch,
                        self.main.rxdata.att.yaw,
                    ),
                    0, # index
                    1, # success
                    b''
                )

        if self._last_cam_beat:
            if self.main.rxdata.time.time-self._last_cam_beat > HEARTBEAT_TIMEOUT*1e6:
                self._mavlogger.log(MAVLOG_LOG, f"Heartbeat timeout from camera #{target}, closing...")
                self._last_cam_beat = False
                self._cam_conn.close()
                import common.key as key
                asyncio.create_task(self._establish_cam(key=key.CAMKEY.encode('utf-8')))

        await asyncio.sleep(0)

    async def _rx(self) -> None:
        """Recieve messages from GCS over mavlink."""
        logger.debug("Starting CommManager (RX)")
        await self._comm_rx_loop()

    async def _rxcam(self) -> None:
        """Recieve messages from camera over mavlink."""
        logger.debug("Starting camera (RX)")
        await self._comm_rxcam_loop()

    @async_loop_decorator(close=False)
    async def _comm_tx_loop(self) -> None:
        """Transmit continuous messages."""
        await asyncio.sleep(0)

    async def _tx(self) -> None:
        """Transmit continuous messages."""
        logger.debug("Starting CommManager (TX)")
        await self._comm_tx_loop()

    async def _roi_calc(self) -> None:
        try:
            logger.info("Starting ROI cycle")
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
                    p = math.radians(p)-self.main.rxdata.att.pitch
                    quat = euler_to_quaternion(0.0, p, y)
                    self._cam_conn.mav.gimbal_device_set_attitude_send(
                        self._cam_id,
                        m.MAV_COMP_ID_CAMERA,
                        m.GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME,
                        quat,
                        0.0, 0.0, 0.0 # angular velocities
                    )
                    await asyncio.sleep(1 / self._txfreq)
                except asyncio.exceptions.CancelledError:
                    break
                except Exception as e:
                    logger.error(f"Error in ROI cycle: {e}")
                    raise
        finally:
            logger.info("Closing ROI cycle")

    @async_loop_decorator(close=False)
    async def _comm_heartbeat_loop(self) -> None:
        msg = [
            m.MAV_TYPE_VTOL_RESERVED4, # 24
            m.MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY,
            int(self.main.state.mode),
            int(self.main.state.custom_mode),
            int(self.main.state.state)
        ]

        try:
            self._mav_conn_gcs.mav.heartbeat_send(*msg)
        except AttributeError:
            pass

        def uint8(num):
            return int(min(max(num, 0), 255))

        self._mav_conn_gcs.mav.high_latency2_send(
            int(self.main.rxdata.time.time * 1e-3),
            m.MAV_TYPE_VTOL_RESERVED4,
            m.MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY,
            self.main.state.custom_mode,
            int(self.main.rxdata.gps.latitude * 1e7),
            int(self.main.rxdata.gps.longitude * 1e7),
            int(self.main.rxdata.gps.altitude),
            int(self.main.afcs._sp_altitude),
            uint8(math.degrees(self.main.rxdata.att.yaw) * 0.5),
            uint8(math.degrees(self.main.afcs._sp_heading) * 0.5),
            int(min(self.main.navigator.distance * 1e-1, 65535)),
            uint8(2.387324 * self.main.txdata.esc1.kinematics.angular_velocity.radian_per_second
                               + self.main.txdata.esc2.kinematics.angular_velocity.radian_per_second
                               + self.main.txdata.esc3.kinematics.angular_velocity.radian_per_second
                               + self.main.txdata.esc4.kinematics.angular_velocity.radian_per_second / self.main.afcs.MAX_THROTTLE),
            uint8(self.main.rxdata.ias.ias * 5),
            uint8(self.main.afcs._spf_ias * 5),
            uint8(self.main.rxdata.gps.yspeed * 5),
            0, # windspeed
            0, # wind heading
            0, # h error
            0, # v error
            0, # air temp
            int(min(max(-self.main.rxdata.gps.dspeed * 1e1, -128), 127)),
            100, # battery! TODO
            1, # waypoint number TODO
            0, # failure flags TODO
            self.main.state.custom_submode, 
            int(min(max(self.main.rxdata.alt.altitude * 0.1 - 128, -128), 127)), 
            0 # custom placeholder
        )

        try:
            self._cam_conn.mav.heartbeat_send(*msg)
            if self.main.state.custom_submode in [g.CUSTOM_SUBMODE_TAKEOFF_ASCENT, g.CUSTOM_SUBMODE_LANDING_HOVER, g.CUSTOM_SUBMODE_LANDING_DESCENT]:
                self._cam_conn.mav.gimbal_device_set_attitude_send(
                    self._cam_id,
                    m.MAV_COMP_ID_CAMERA,
                    m.GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME,
                    euler_to_quaternion(0.0, math.radians(-90), math.radians(0)),
                    0.0, 0.0, 0.0 # angular velocities
                )
                if not self._count%5:
                    self._cam_conn.mav.command_long_send(
                        self._cam_id,
                        m.MAV_COMP_ID_CAMERA,
                        m.MAV_CMD_IMAGE_START_CAPTURE,
                        0,
                        0, 1, 1, 0,
                        0, 0, 0
                    )
                self._count += 1
        except AttributeError:
            pass

        self._mavlogger.log(MAVLOG_DEBUG, "TX Heartbeat")
        await asyncio.sleep(1 / self._heartbeatfreq)

    async def _heartbeat(self) -> None:
        """Periodically publish a heartbeat message."""
        logger.debug("Starting CommManager (Heartbeat)")
        self._count = 1
        await self._comm_heartbeat_loop()

    async def close(self) -> None:
        """Close the instance."""
        self._mav_conn_gcs.close()

        import common.key as key
        self._cam_conn.mav.change_operator_control_send(self.main.config.getint('mavlink_ids', 'cam_id'), 1, 0, key.CAMKEY.encode('utf-8'))
        self._cam_conn.close()
        logger.info("Closing CommManager")


class Main:
    """Main class for handling the operation of a system.

    Parameters
    ----------
    systemid : int, optional
        The system ID for this instance (default is 1).
    config : str, optional
        Config file path (default is './common/config.ini').

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
    rxdata : RxBuffer
        Global receive data.
    txdata : TxBuffer
        Global transmit data.
    state : g
        State information.
    
    Methods
    -------
    run(self, graph: str = None)
        Run the UAV instance.
    """

    def __init__(self, config: str = './common/config.ini') -> None:
        """Initialize a Main instance.

        Parameters
        ----------
        systemid : int, optional
            The system ID for this instance (default is 1).
        config : str, optional
            The path to the configuration file (default is './common/config.ini').

        Raises
        ------
        AssertionError
            If the system ID is not a positive integer <= 255.
        """

        self.config = ConfigParser()
        self.config.read(config)

        self.systemid = self.config.getint('mavlink_ids', 'uav_id')
        assert isinstance(self.systemid, int) and self.systemid > 0 and self.systemid.bit_length() <= 8, "System ID in config file must be UINT8"

        self.boot = asyncio.Event()
        self.stop = asyncio.Event()

        self.rxdata = RxBuffer()
        self.txdata = TxBuffer()

        self.state = StateManager(m.MAV_STATE_UNINIT, m.MAV_MODE_PREFLIGHT, g.CUSTOM_MODE_UNINIT, g.CUSTOM_SUBMODE_UNINIT, self.boot)

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
                    logger.error(f'Error in Grapher: {e}')
        except KeyboardInterrupt:
            self.stop.set()
            raise
        finally:
            self._grapher.close()
            logger.info('Closing Grapher')

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
                        logger.error(f'Error in Printer: {e}')
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

        self.comm = CommManager(self)
        self.io = UAVCANManager(self)
        self.afcs = AFCS(self)
        self.navigator = Navigator(self)

        comm_manager = asyncio.create_task(self.comm.manager())
        await asyncio.sleep(0)

        logger.warning(f"Creating instance #{self.systemid}, waiting for boot command from GCS")

        try:
            if DEBUG_SKIP>-1:
                self.boot.set()
                self.state.inc_mode()
            await self.boot.wait()
        except asyncio.exceptions.CancelledError:
            comm_manager.cancel()
            await asyncio.sleep(0)
            logger.warning(f"Never booted, closing instance #{self.systemid}")
            return

        logger.warning(f"Booting instance #{self.systemid}...")

        boot_tasks = [
            asyncio.create_task(self.comm.boot_proc()),
            asyncio.create_task(self.afcs.boot_proc()),
            asyncio.create_task(self.navigator.boot_proc()),
            asyncio.create_task(self.io.boot_proc()),
        ]

        try:
            await asyncio.gather(*boot_tasks)
            self.state.inc_mode()
        except asyncio.exceptions.CancelledError:
            comm_manager.cancel()
            await asyncio.sleep(0)
            logger.warning(f"Ctrl-C during boot cycle, closing instance #{self.systemid}")
            return

        logger.warning(f"Boot successful on #{self.systemid}")

        for _ in range(DEBUG_SKIP):
            self.state.inc_mode()

        tasks = [
            asyncio.create_task(self.afcs.run()),
            asyncio.create_task(self.navigator.run()),
            asyncio.create_task(self.io.run()),
        ]

        if graph:
            logger.info("Grapher on")
            tasks.append(asyncio.create_task(self._graph(name=graph)))

        if print_:
            logger.info("Printer on")
            tasks.append(asyncio.create_task(self._print(name=print_)))

        try:
            await asyncio.gather(*tasks)
        except asyncio.exceptions.CancelledError:
            self.stop.set()
            logger.warning(f"Closing instance #{self.systemid}...")

        close_tasks = [
            asyncio.create_task(self.afcs.close()),
            asyncio.create_task(self.comm.close()),
            asyncio.create_task(self.io.close()),
        ]

        try:
            await asyncio.gather(*close_tasks)
        except asyncio.exceptions.CancelledError:
            logger.error("Instance closed prematurely")
        else:
            logger.info("Instance closed")


async def main(graph: str | bool = False, print_: str | bool = False) -> None:
    instance = Main()
    await instance.run(graph=graph, print_=print_)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-g", "--graph", nargs='?', default=False, const='rxdata.att.rollspeed', help="Attribute to graph")
    parser.add_argument("-p", "--print", nargs='?', default=False, const='afcs._throttles', help="Attribute to print")
    parser.add_argument("-s", "--skip", nargs='?', default='-1', const='0', help="Skip number of modes on startup")
    args = parser.parse_args()

    whitelisted = frozenset("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789._-[]")
    if args.graph:
        assert all(char in whitelisted for char in args.graph) and '__' not in args.graph
    if args.print:
        assert all(char in whitelisted for char in args.print) and '__' not in args.print
    if args.skip:
        assert all(char in whitelisted for char in args.skip) and '__' not in args.skip

    DEBUG_SKIP = int(args.skip)

    asyncio.run(main(args.graph, args.print))
