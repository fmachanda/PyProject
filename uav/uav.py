"""UAV Main

This script connects to simulated hardware through realistic UAVCAN and 
MAVLINK protocols. The script recieves data from various sensors and
publishes desired servo and throttle settings. A method to connect with 
a GCS over MAVLINK is provided. The UAV is designed to be autonomous.

This script uses values from the common/CONFIG.ini file.

See https://github.com/fmachanda/fmuas-main for more details.
"""

import asyncio
import logging
import math
import os
import sys
from configparser import ConfigParser
from typing import Any
import numpy as np

if os.path.basename(os.getcwd()) == 'uav':
    os.chdir('..')
sys.path.append(os.getcwd())
os.environ['CYPHAL_PATH'] = './common/data_types/custom_data_types;./common/data_types/public_regulated_data_types'
os.environ['PYCYPHAL_PATH'] = './common/pycyphal_generated'
os.environ['MAVLINK20'] = '1'

import pycyphal
import pycyphal.application
import uavcan
import uavcan_archived
from pymavlink import mavutil
from uavcan_archived.equipment import actuator, ahrs, air_data, esc, gnss, range_sensor

from common.pid import PID
from common.state_manager import GlobalState as g

m = mavutil.mavlink

logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.INFO)
logging.getLogger('pymavlink').setLevel(logging.ERROR)

os.system('cls' if os.name == 'nt' else 'clear')

DEFAULT_FREQ = 60

system_ids = []


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

        @staticmethod
        def quaternion_to_euler(q: tuple[float, float, float, float]) -> tuple[float, float, float]:
            """Convert a quaternion angle to Euler. 

            Copied on 10/2/2023 from automaticaddison.com
            
            Parameters
            ----------
            q : tuple
                The quaternion values x, y, z, w
            
            Returns
            -------
            tuple
                A tuple of roll, pitch, yaw as radians in Euler format
            
            Raises
            ------
            ValueError
                If the input is not of length 4
            """

            if len(q)!=4:
                raise ValueError('Quaternion input must be tuple with length 4')

            x, y, z, w = q

            t0 = +2.0 * (w * x + y * z)
            t1 = +1.0 - 2.0 * (x * x + y * y)
            roll = math.atan2(t0, t1)
        
            t2 = +2.0 * (w * y - z * x)
            t2 = +1.0 if t2 > +1.0 else t2
            t2 = -1.0 if t2 < -1.0 else t2
            pitch = math.asin(t2)
        
            t3 = +2.0 * (w * z + x * y)
            t4 = +1.0 - 2.0 * (y * y + z * z)
            yaw = math.atan2(t3, t4)
        
            return roll, pitch, yaw # in radians

        def dump(self, msg: ahrs.Solution_1) -> None:
            """Store data from a message."""
            self._last_time = self.time
            # self._last_pitch = self.pitch
            # self._last_pitchspeed = self.pitchspeed
            # self._last_roll = self.roll
            # self._last_rollspeed = self.rollspeed
            # self._last_yaw = self.yaw
            # self._last_yawspeed = self.yawspeed

            self.time = msg.timestamp.usec
            self.roll, self.pitch, self.yaw = GlobalRx.Att.quaternion_to_euler(msg.orientation_xyzw)

            if self.yaw < 0:
                self.yaw += 2*math.pi

            self.rollspeed, self.pitchspeed, self.yawspeed = msg.angular_velocity
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

        def dump(self, msg: range_sensor.Measurement_1) -> None:
            """Store data from a message."""
            self._last_time = self.time
            self._last_altitude = self.altitude
            self._last_vs = self.vs

            self.time = msg.timestamp.usec
            self.altitude = msg.range_ # 'range' becomes 'range_'

            try:
                self.vs = (self.altitude - self._last_altitude) / (self.time - self._last_time)
            except ZeroDivisionError:
                self.vs = self._last_vs

            self.dt = self.time - self._last_time

    class Gps:
        """Store GPS data."""
        def __init__(self) -> None:
            self.time = 0.0
            self.latitude = 0.0
            self.longitude = 0.0
            self.altitude = 0.0
            self.nspeed = 0.0
            self.espeed = 0.0
            self.dspeed = 0.0

            self._last_time = 0.0
            self.dt = 0.0
            # self._last_latitude = 0.0
            # self._last_longitude = 0.0
            # self._last_altitude = 0.0
            # self._last_nspeed = 0.0
            # self._last_espeed = 0.0
            # self._last_dspeed = 0.0

        def dump(self, msg: gnss.Fix2_1) -> None:
            """Store data from a message."""
            self._last_time = self.time
            # self._last_latitude = self.latitude
            # self._last_longitude = self.longitude
            # self._last_altitude = self.altitude
            # self._last_nspeed = self.nspeed
            # self._last_espeed = self.espeed
            # self._last_dspeed = self.dspeed

            self.time = msg.timestamp.usec
            self.latitude = msg.latitude_deg_1e8# / 1e8
            self.longitude = msg.longitude_deg_1e8# / 1e8
            self.altitude = msg.height_msl_mm / 1e3
            self.nspeed, self.espeed, self.dspeed= msg.ned_velocity

            self.dt = self.time - self._last_time

    class Ias:
        """Store airspeed data."""
        def __init__(self) -> None:
            self.time = 0.0
            self.ias = 0.0

            self._last_time = 0.0
            self.dt = 0.0
            # self._last_ias = 0.0

        def dump(self, msg: air_data.IndicatedAirspeed_1, time: 'GlobalRx.Time.time') -> None:
            """Store data from a message."""
            self._last_time = self.time
            # self._last_ias = self.ias
            
            self.time = time
            self.ias = msg.indicated_airspeed

            self.dt = self.time - self._last_time

    class Aoa:
        """Store AOA data."""
        def __init__(self) -> None:
            self.time = 0.0
            self.aoa = 0.0

            self._last_time = 0.0
            self.dt = 0.0
            # self._last_aoa = 0.0

        def dump(self, msg: air_data.AngleOfAttack_1, time: 'GlobalRx.Time.time') -> None:
            """Store data from a message."""
            self._last_time = self.time
            # self._last_aoa = self.aoa
            
            self.time = time
            self.aoa = msg.aoa

            self.dt = self.time - self._last_time

    class Slip:
        """Store sideslip data."""
        def __init__(self) -> None:
            self.time = 0.0
            self.slip = 0.0

            self._last_time = 0.0
            self.dt = 0.0
            # self._last_slip = 0.0

        def dump(self, msg: air_data.Sideslip_1, time: 'GlobalRx.Time.time') -> None:
            """Store data from a message."""
            self._last_time = self.time
            # self._last_slip = self.slip
            
            self.time = time
            self.slip = msg.sideslip_angle

            self.dt = self.time - self._last_time

    def __init__(self) -> None:
        self.time = GlobalRx.Time()
        self.att =  GlobalRx.Att()
        self.alt =  GlobalRx.Alt()
        self.gps =  GlobalRx.Gps()
        self.ias =  GlobalRx.Ias()
        self.aoa =  GlobalRx.Aoa()
        self.slip = GlobalRx.Slip()


class GlobalTx:
    """Store control data before publishing."""
    def __init__(self) -> None:
        self.servo = actuator.ArrayCommand_1([
            actuator.Command_1(0, actuator.Command_1.COMMAND_TYPE_POSITION, 0.0), # Elevon 1
            actuator.Command_1(1, actuator.Command_1.COMMAND_TYPE_POSITION, 0.0), # Elevon 2
            actuator.Command_1(2, actuator.Command_1.COMMAND_TYPE_POSITION, 0.0), # Rudder
            actuator.Command_1(3, actuator.Command_1.COMMAND_TYPE_POSITION, 0.0)  # Wing-tilt
        ])
        
        self.esc = esc.RawCommand_1([
            int(0.0 * 8191), # Throttle 1
            int(0.0 * 8191), # Throttle 2
            int(0.0 * 8191), # Throttle 3
            int(0.0 * 8191)  # Throttle 4
        ])


# Declaration to avoid flagged error in Navigator class
class Waypoint: ...


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

    def __init__(self, main:'Main', freq: int = DEFAULT_FREQ) -> None:
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

        logging.info('Initializing UAVCAN Node...')

        self._registry = pycyphal.application.make_registry(environment_variables={
            'UAVCAN__NODE__ID'                      :self.main.config.get('node_ids', 'mainio'),
            'UAVCAN__UDP__IFACE'                    :self.main.config.get('main', 'udp'),

            'UAVCAN__SUB__ATTITUDE__ID'             :self.main.config.get('subject_ids', 'attitude'),
            'UAVCAN__SUB__ALTITUDE__ID'             :self.main.config.get('subject_ids', 'altitude'),
            'UAVCAN__SUB__GPS__ID'                  :self.main.config.get('subject_ids', 'gps'),

            'UAVCAN__SUB__IAS__ID'                  :self.main.config.get('subject_ids', 'ias'),
            'UAVCAN__SUB__AOA__ID'                  :self.main.config.get('subject_ids', 'aoa'),
            'UAVCAN__SUB__SLIP__ID'                 :self.main.config.get('subject_ids', 'slip'),

            'UAVCAN__PUB__SERVOS__ID'               :self.main.config.get('subject_ids', 'servo'),
            'UAVCAN__PUB__ESCS__ID'                 :self.main.config.get('subject_ids', 'esc'),

            'UAVCAN__SUB__CLOCK_SYNC_TIME__ID'      :self.main.config.get('subject_ids', 'clock_sync_time'),
            # 'UAVCAN__SUB__CLOCK_SYNC_TIME_LAST__ID' :self.main.config.get('subject_ids', 'clock_sync_time_last'),

            # 'UAVCAN__SUB__GPS_SYNC_TIME__ID'        :self.main.config.get('subject_ids', 'gps_sync_time'),
            # 'UAVCAN__SUB__GPS_SYNC_TIME_LAST__ID'   :self.main.config.get('subject_ids', 'gps_sync_time_last'),

            # 'UAVCAN__CLN__CLOCK_SYNC_INFO__ID'      :self.main.config.get('service_ids', 'clock_sync_info'),
            # 'UAVCAN__CLN__GPS_SYNC_INFO__ID'        :self.main.config.get('service_ids', 'gps_sync_info'),
        })
        
        node_info = uavcan.node.GetInfo_1.Response(
            software_version=uavcan.node.Version_1(major=1, minor=0),
            name=f'fmuas.uasmain{self.main.systemid}.mainio',
        )

        self._node = pycyphal.application.make_node(node_info, self._registry)

        self._node.heartbeat_publisher.mode = uavcan.node.Mode_1.INITIALIZATION
        self._node.heartbeat_publisher.vendor_specific_status_code = os.getpid() % 100
        
        self._sub_att = self._node.make_subscriber(ahrs.Solution_1, 'attitude')
        self._sub_alt = self._node.make_subscriber(range_sensor.Measurement_1, 'altitude')
        self._sub_gps = self._node.make_subscriber(gnss.Fix2_1, 'gps')

        self._sub_ias = self._node.make_subscriber(air_data.IndicatedAirspeed_1, 'ias')
        self._sub_aoa = self._node.make_subscriber(air_data.AngleOfAttack_1, 'aoa')
        self._sub_slip = self._node.make_subscriber(air_data.Sideslip_1, 'slip')

        self._pub_servos = self._node.make_publisher(actuator.ArrayCommand_1, 'servos')
        self._pub_escs = self._node.make_publisher(esc.RawCommand_1, 'escs')

        self._sub_clock_sync_time = self._node.make_subscriber(uavcan.time.SynchronizedTimestamp_1, 'clock_sync_time')
        # self._sub_clock_sync_time_last = self._node.make_subscriber(uavcan.time.Synchronization_1, 'clock_sync_time_last')

        # self._sub_gps_sync_time = self._node.make_subscriber(uavcan.time.SynchronizedTimestamp_1, 'gps_sync_time')
        # self._sub_gps_sync_time_last = self._node.make_subscriber(uavcan.time.Synchronization_1, 'gps_sync_time_last')

        # self._cln_clock_sync_info = self._node.make_client(uavcan.time.GetSynchronizationMasterInfo_0, self.main.config.getint('service_ids', 'clock_sync_info'), 'clock_sync_info')
        # self._cln_gps_sync_info = self._node.make_client(uavcan.time.GetSynchronizationMasterInfo_0, self.main.config.getint('service_ids', 'gps_sync_info'), 'gps_sync_info')

        self._node.start()

        logging.info('MainIO initialized')

    async def boot(self) -> None:
        """Perform boot-related tasks."""
        # TODO: do boot stuff here, maybe read a different .ini?
        await asyncio.sleep(0)

    async def run(self) -> None:
        """Recieve subscripted data and write to publishers."""
        # region Subscriptions
        def on_time(msg: uavcan.time.SynchronizedTimestamp_1, _: pycyphal.transport.TransferFrom) -> None:
            self.main.rxdata.time.dump(msg)
        self._sub_clock_sync_time.receive_in_background(on_time)

        def on_att(msg: ahrs.Solution_1, _: pycyphal.transport.TransferFrom) -> None:
            self.main.rxdata.att.dump(msg)
        self._sub_att.receive_in_background(on_att)

        def on_alt(msg: range_sensor.Measurement_1, _: pycyphal.transport.TransferFrom) -> None:
            self.main.rxdata.alt.dump(msg)
        self._sub_alt.receive_in_background(on_alt)

        def on_gps(msg: gnss.Fix2_1, _: pycyphal.transport.TransferFrom) -> None:
            self.main.rxdata.gps.dump(msg)
        self._sub_gps.receive_in_background(on_gps)

        def on_ias(msg: air_data.IndicatedAirspeed_1, _: pycyphal.transport.TransferFrom) -> None:
            t = self.main.rxdata.time.time
            self.main.rxdata.ias.dump(msg, t)
        self._sub_ias.receive_in_background(on_ias)

        def on_aoa(msg: air_data.AngleOfAttack_1, _: pycyphal.transport.TransferFrom) -> None:
            t = self.main.rxdata.time.time
            self.main.rxdata.aoa.dump(msg, t)
        self._sub_aoa.receive_in_background(on_aoa)

        def on_slip(msg: air_data.Sideslip_1, _: pycyphal.transport.TransferFrom) -> None:
            t = self.main.rxdata.time.time
            self.main.rxdata.slip.dump(msg, t)
        self._sub_slip.receive_in_background(on_slip)
        # endregion
        
        self._node.heartbeat_publisher.mode = uavcan.node.Mode_1.OPERATIONAL

        logging.warning('UAVCAN Node Running...\n----- Ctrl-C to exit -----') # TODO: no verification

        try:
            while not self.main.stop.is_set():
                try:
                    await self._pub_servos.publish(self.main.txdata.servo)
                    await self._pub_escs.publish(self.main.txdata.esc)
                    await asyncio.sleep(1 / self._freq)
                except asyncio.exceptions.CancelledError:
                    self.main.stop.set()
                    raise
                except Exception as e:
                    logging.error(f'Error in MainIO: {e}')
        except KeyboardInterrupt:
            self.main.stop.set()
            raise
        finally:
            self.close()

    def close(self) -> None:
        """Close the instance."""
        logging.info('Closing MainIO') # TODO: Change to logging.debug()
        self._node.close()


class Navigator:
    """Converts navigational commands to autopilot commands."""
    class Waypoint:
        """Stores information for a waypoint."""
        count = 0
        def __init__(self, latitude: float, longitude: float, altitude: float = None, name: str = None) -> None:
            self.latitude = latitude
            self.longitude = longitude
            self.altitude = altitude

            Waypoint.count += 1
            self.name = name if name is not None else f'Waypoint {Waypoint.count}'

    def __init__(self, main: 'Main') -> None:
        """Initializes the Navigator class.

        Parameters
        ----------
        main : 'Main'
            The main object.
        """

        self.main = main
        self._waypoint_list = []

    async def boot(self) -> None:
        """Perform boot-related tasks."""
        # TODO: do boot stuff here, calibrate gps etc
        # await gps online
        self.add_wpt(Waypoint(self.main.rxdata.gps.latitude, self.main.rxdata.gps.latitude, self.main.rxdata.gps.altitude, name='Start'))
        await asyncio.sleep(0)

    def add_wpt(self, wpt: Waypoint) -> None:
        self._waypoint_list.append(wpt)

    def calc_heading(self) -> None:
        pass

    async def run(self) -> None:
        pass


class Processor:
    """Manages various calculations and controls for the system.

    This class handles the calculation of servo commands and throttle 
    outputs based on sensor data and control setpoints.

    Attributes
    ----------
    main : 'Main'
        The main object representing the core of the system.
    spf_altitude : float
        Setpoint for altitude.
    spf_heading : float
        Setpoint for heading.
    spf_ias : float
        Setpoint for indicated airspeed.

    Methods
    -------
    boot(self)
        Initialize and perform boot-related tasks.
    _calc_dyaw(self, value, setpoint)
        Calculate yaw error.
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

        self._fservos = np.zeros(4, dtype=np.float16) # elevons*2, rudder, wingtilt
        self._vservos = np.zeros(4, dtype=np.float16)

        self._fthrottles = np.zeros(4, dtype=np.float16) # throttles*4
        self._vthrottles = np.zeros(4, dtype=np.float16)

        self.spf_altitude = 0.0
        self.spf_heading = 0.0
        self.spf_ias = 0.0

        self._spf_vpath = 0.0
        self._spf_aoa = 0.1
        self._spf_roll = 0.0
        self._spf_rollspeed = 0.0

        self._outf_pitch = 0.0
        self._outf_roll = 0.0
        self._outf_yaw = 0.0
        self._outf_throttle = 0.0

        # self._pid{f or v}_{from}_{to}

        self._pidf_alt_vpa = PID(kp=0.0, td=0.0, ti=0.0, integral_limit=0.05, maximum=0.05, minimum=-0.05)
        self._pidf_vpa_aoa = PID(kp=0.7, ti=3.0, td=0.1, integral_limit=0.2, maximum=0.15, minimum=-0.05)

        self._pidf_aoa_out = PID(kp=-0.07, ti=-0.008, td=0.02, integral_limit=1, maximum=0.0, minimum=-math.pi/12)
        # self._pidf_slp_out = PID(kp=-0.03, ti=-0.001, td=3.0, integral_limit=math.pi/6, maximum=1.0, minimum=-1.0)
        self._pidf_slp_out = PID(kp=0.0, ti=0.0, td=0.0, integral_limit=math.pi/6, maximum=1.0, minimum=-1.0)
        self._pidf_ias_out = PID(kp=0.0, ti=0.0, td=0.0, integral_limit=0.25, maximum=0.25, minimum=0.02)

        self._pidf_yaw_rol = PID(kp=-0.8, ti=-8.0, td=0.05, integral_limit=0.1, maximum=math.pi/6, minimum=-math.pi/6)

        self._pidf_rol_rls = PID(kp=2.0, ti=6.0, td=0.08, integral_limit=0.2, maximum=2.0, minimum=-2.0)
        self._pidf_rls_out = PID(kp=0.008, ti=0.003, td=0.01, integral_limit=0.1, maximum=0.1, minimum=-0.1)
        # self._pidf_rls_out = PID(kp=0.015, ti=0.35, td=0.001, integral_limit=0.1, maximum=0.1, minimum=-0.1)

    async def boot(self) -> None:
        """Perform boot-related tasks."""
        # TODO: do boot stuff here, maybe read a different .ini?
        self.main.state.inc_mode()
        await asyncio.sleep(0)

    # region Calculations
    @staticmethod
    def _calc_dyaw(value: float, setpoint: float) -> float: # TODO: if negative radians sent by uavcan
        """Calculate yaw error.

        Calculates the yaw error based on a given value and setpoint, 
        taking into account wraparound at +/- pi radians.

        Parameters
        ----------
        value : float
            Current yaw value.
        setpoint : float
            Desired yaw setpoint.

        Returns
        -------
        float
            Yaw error.
        """

        dyaw = value - setpoint
        dyaw = dyaw - 2*math.pi if dyaw > math.pi else dyaw
        dyaw = dyaw + 2*math.pi if dyaw <= -math.pi else dyaw
        return -dyaw

    def _flight_servos(self) -> np.ndarray:
        """Calculate flight servo commands from sensors."""
        if self.main.rxdata.alt.dt > 0.0:
            self._spf_vpath = self._pidf_alt_vpa.cycle(self.main.rxdata.alt.altitude, self.spf_altitude, self.main.rxdata.alt.dt)
            self.main.rxdata.alt.dt = 0.0

        if self.main.rxdata.att.dt > 0.0 or self.main.rxdata.aoa.dt > 0.0:
            if not self.main.rxdata.aoa.dt > 0.0:
                dt = self.main.rxdata.att.dt
            elif not self.main.rxdata.att.dt > 0.0:
                dt = self.main.rxdata.aoa.dt
            else:
                dt = (self.main.rxdata.att.dt + self.main.rxdata.aoa.dt) / 2

            self._vpath = self.main.rxdata.att.pitch - math.cos(self.main.rxdata.att.roll) * self.main.rxdata.aoa.aoa
            self._spf_aoa = self._pidf_vpa_aoa.cycle(self._vpath, self._spf_vpath, dt) # TODO

        if self.main.rxdata.att.dt > 0.0:
            self._dyaw = Processor._calc_dyaw(self.main.rxdata.att.yaw, self.spf_heading)

            self._spf_roll = self._pidf_yaw_rol.cycle(self._dyaw, 0.0, self.main.rxdata.att.dt)
            self._spf_rollspeed = self._pidf_rol_rls.cycle(self.main.rxdata.att.roll, self._spf_roll, self.main.rxdata.att.dt)
            self._outf_roll = self._pidf_rls_out.cycle(self.main.rxdata.att.rollspeed, self._spf_rollspeed, self.main.rxdata.att.dt)

            self.main.rxdata.att.dt = 0.0

        if self.main.rxdata.aoa.dt > 0.0:
            self._outf_pitch = self._pidf_aoa_out.cycle(self.main.rxdata.aoa.aoa, self._spf_aoa, self.main.rxdata.aoa.dt)
            self.main.rxdata.aoa.dt = 0.0

        if self.main.rxdata.slip.dt > 0.0:
            self._outf_yaw = 0.0 #self._pidf_slp_out.cycle(self.main.rxdata.slip.slip, 0.0, self.main.rxdata.slip.dt)
            self.main.rxdata.slip.dt = 0.0

        self._fservos[0] = self._outf_pitch + self._outf_roll # TODO
        self._fservos[1] = self._outf_pitch - self._outf_roll # TODO
        self._fservos[2] = self._outf_yaw

        return self._fservos

    def _vtol_servos(self) -> np.ndarray:
        """Calculate VTOL servo commands from sensors."""
        
        return self._vservos

    def _flight_throttles(self) -> np.ndarray:
        """Calculate flight throttle commands from sensors."""
        if self.main.rxdata.ias.dt > 0.0:
            self._outf_throttle = self._pidf_ias_out.cycle(self.main.rxdata.ias.ias, self.spf_ias, self.main.rxdata.ias.dt)
            self.main.rxdata.ias.dt = 0.0

        self._fthrottles.fill(self._outf_throttle)

        return self._fthrottles

    def _vtol_throttles(self) -> np.ndarray:
        """Calculate VTOL throttle commands from sensors."""
        return self._vthrottles
    # endregion

    async def run(self) -> None:
        """Calculate desired control positions."""
        logging.info('Starting Processor')

        try:
            while not self.main.stop.is_set():
                try:
                    if self.main.state.custom_mode in [g.CUSTOM_MODE_TAKEOFF, g.CUSTOM_MODE_LANDING]:
                        # TODO: MIXING PLACEHOLDER vvv
                        self._servos = (1296 / (self.main.rxdata.ias.ias**2)) * np.sum(np.array([1.0 * self._flight_servos(), 0.0 * self._vtol_servos()]), axis=0, dtype=np.float16)
                        self._throttles = np.sum(np.array([1.0 * self._flight_throttles(), 0.0 * self._vtol_throttles()]), axis=0, dtype=np.float16)
                    elif self.main.state.custom_mode == g.CUSTOM_MODE_FLIGHT and self.main.state.custom_submode != g.CUSTOM_SUBMODE_FLIGHT_MANUAL:
                        self._servos = self._flight_servos()
                        self._throttles = self._flight_throttles()
                    else:
                        self._servos = np.zeros(4, dtype=np.float16).fill(0.0)
                        self._throttles = np.zeros(4, dtype=np.float16).fill(0.0)

                    self.main.txdata.servo = actuator.ArrayCommand_1([
                        actuator.Command_1(0, actuator.Command_1.COMMAND_TYPE_POSITION, self._servos[0]), # Elevon 1
                        actuator.Command_1(1, actuator.Command_1.COMMAND_TYPE_POSITION, self._servos[1]), # Elevon 2
                        actuator.Command_1(2, actuator.Command_1.COMMAND_TYPE_POSITION, self._servos[2]), # Rudder
                        actuator.Command_1(3, actuator.Command_1.COMMAND_TYPE_POSITION, self._servos[3])  # Wing-tilt
                    ])
                    
                    self.main.txdata.esc = esc.RawCommand_1([
                        int(self._throttles[0] * 8191), # Throttle 1
                        int(self._throttles[1] * 8191), # Throttle 2
                        int(self._throttles[2] * 8191), # Throttle 3
                        int(self._throttles[3] * 8191)  # Throttle 4
                    ])

                    try:
                        await asyncio.sleep(0)
                    except asyncio.exceptions.CancelledError:
                        self.main.stop.set()
                        raise
                except Exception as e:
                    logging.error(f'Error in Processor: {e}')
        except KeyboardInterrupt:
            self.main.stop.set()
            raise
        finally:
            logging.info('Closing Processor') # TODO: Change to logging.debug()


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

        self._gcs_id = None
        self._mav_conn_gcs: mavutil.mavfile = mavutil.mavlink_connection(self.main.config.get('mavlink', 'uav_gcs_conn'), source_system=self.main.systemid, source_component=m.MAV_COMP_ID_AUTOPILOT1, input=False, autoreconnect=True)
        import common.key as key
        self._mav_conn_gcs.setup_signing(key.KEY.encode('utf-8'))

    async def manager(self) -> None:
        """Manage the controller's various operations."""
        asyncio.create_task(self._heartbeat())
        asyncio.create_task(self._rx())
        asyncio.create_task(self._tx())

        logging.info('Starting Controller')

        try:
            while not self.main.stop.is_set():
                try:
                    # self.main.boot.set()
                    await asyncio.sleep(0)
                except asyncio.exceptions.CancelledError:
                    self.main.stop.set()
                    raise
                except Exception as e:
                    logging.error(f'Error in Controller: {e}')
        except KeyboardInterrupt:
            self.main.stop.set()
            raise
        finally:
            self.close()

    async def _rx(self) -> None:
        """Recieve messages from GCS over mavlink."""
        """
        'if' statements structure
        ├── HEARTBEAT
        ├── BAD_DATA
        └── * Target messages
            ├── CHANGE_OPERATOR_CONTROL
            └── * GCS messages
                ├── COMMAND_LONG
                    ├── DO_SET_MODE
                    ├── DO_CHANGE_ALTITUDE
                    ├── DO_CHANGE_SPEED
                    ├── * Command int only
                    └── * Command unsupported
                └── COMMAND_INT
                    ├── DO_REPOSITION
                    ├── NAV_TAKEOFF
                    ├── NAV_VTOL_TAKEOFF
                    ├── * Command long only
                    └── * Command unsupported
        """
        logging.debug('Starting Controller (RX)')

        try:
            i=1
            while not self.main.stop.is_set():
                try:
                    try:
                        msg = self._mav_conn_gcs.recv_msg()
                    except ConnectionError:
                        try:
                            logging.debug('Controller (rx) connection refused')
                            await asyncio.sleep(0)
                        except asyncio.exceptions.CancelledError:
                            self.main.stop.set()
                            raise
                        finally:
                            continue

                    if msg is not None:
                        # HEARTBEAT
                        if msg.get_type() == 'HEARTBEAT':
                            logging.debug(f'Heartbeat message from link #{msg.get_srcSystem()}')
                        # BAD_DATA
                        elif msg.get_type() == 'BAD_DATA':
                            pass
                        # Target messages
                        elif msg.target_system == self.main.systemid:
                            # CHANGE_OPERATOR_CONTROL
                            if msg.get_type() == 'CHANGE_OPERATOR_CONTROL':
                                import common.key as key
                                if msg.control_request == 0 and msg.passkey==key.KEY:
                                    if self._gcs_id is None or self._gcs_id==msg.get_srcSystem():
                                        # Accepted
                                        if self._gcs_id!=msg.get_srcSystem():
                                            logging.info(f'Accepting control request from GCS ({msg.get_srcSystem()})')
                                        self._gcs_id = msg.get_srcSystem()
                                        self._mav_conn_gcs.mav.change_operator_control_ack_send(msg.get_srcSystem(), msg.control_request, 0)
                                    else:
                                        # Already controlled
                                        logging.info(f'Rejecting second control request from {msg.get_srcSystem()}')
                                        self._mav_conn_gcs.mav.change_operator_control_ack_send(msg.get_srcSystem(), msg.control_request, 3)
                                elif msg.control_request == 1 and msg.passkey==key.KEY:# and self._gcs_id is not None:
                                    # Accepted (released)
                                    logging.info(f'Releasing from GCS ({self._gcs_id})')
                                    self._gcs_id = None
                                    self._mav_conn_gcs.mav.change_operator_control_ack_send(msg.get_srcSystem(), msg.control_request, 0)
                                else:
                                    # Bad key
                                    logging.info(f'Bad key in GCS control request')
                                    self._mav_conn_gcs.mav.change_operator_control_ack_send(msg.get_srcSystem(), msg.control_request, 1)

                            # GCS messages
                            if msg.get_srcSystem() == self._gcs_id:
                                # COMMAND_LONG
                                match msg.get_type():
                                    case 'COMMAND_LONG':
                                        match msg.command:
                                            # DO_SET_MODE
                                            case m.MAV_CMD_DO_SET_MODE:
                                                logging.info('Mode change requested')
                                                    
                                                if self.main.state.set_mode(msg.param1, msg.param2, msg.param3): # TODO: Add safety check to verify message like below
                                                    self._mav_conn_gcs.mav.command_ack_send(m.MAV_CMD_DO_SET_MODE, m.MAV_RESULT_ACCEPTED, 255, 0, 0, 0)
                                                else:
                                                    self._mav_conn_gcs.mav.command_ack_send(m.MAV_CMD_DO_SET_MODE, m.MAV_RESULT_DENIED, 255, 0, 0, 0)
                                            # DO_CHANGE_ALTITUDE
                                            case m.MAV_CMD_DO_CHANGE_ALTITUDE:
                                                pass # TODO
                                                try:
                                                    self.main.processor.spf_altitude = msg.param1 # TODO add checks!
                                                    logging.info(f'GCS commanded altitude setpoint to {msg.param1}')
                                                    self._mav_conn_gcs.mav.command_ack_send(m.MAV_CMD_DO_CHANGE_ALTITUDE, m.MAV_RESULT_ACCEPTED, 255, 0, 0, 0)
                                                except AttributeError:
                                                    self._mav_conn_gcs.mav.command_ack_send(m.MAV_CMD_DO_CHANGE_ALTITUDE, m.MAV_RESULT_TEMPORARILY_REJECTED, 255, 0, 0, 0)
                                            # DO_CHANGE_SPEED
                                            case m.MAV_CMD_DO_CHANGE_SPEED:
                                                pass # TODO
                                                try:
                                                    self.main.processor.spf_ias = msg.param2 # TODO add checks!
                                                    logging.info(f'GCS commanded speed setpoint to {msg.param2}')
                                                    self._mav_conn_gcs.mav.command_ack_send(m.MAV_CMD_DO_CHANGE_SPEED, m.MAV_RESULT_ACCEPTED, 255, 0, 0, 0)
                                                except AttributeError:
                                                    self._mav_conn_gcs.mav.command_ack_send(m.MAV_CMD_DO_CHANGE_SPEED, m.MAV_RESULT_TEMPORARILY_REJECTED, 255, 0, 0, 0)
                                            # Command int only
                                            case m.CMD_DO_REPOSITION | m.CMD_NAV_TAKEOFF | m.MAV_CMD_NAV_VTOL_TAKEOFF | m.MAV_CMD_NAV_LAND | m.MAV_CMD_NAV_VTOL_LAND:
                                                self._mav_conn_gcs.mav.command_ack_send(msg.command, m.MAV_RESULT_COMMAND_INT_ONLY, 255, 0, 0, 0)
                                            # Command unsupported
                                            case _:
                                                self._mav_conn_gcs.mav.command_ack_send(msg.command, m.MAV_RESULT_UNSUPPORTED, 255, 0, 0, 0)
                                    case 'COMMAND_INT':
                                        match msg.command:
                                            # DO_REPOSITION
                                            case m.MAV_CMD_DO_REPOSITION: 
                                                pass # TODO
                                            # TODO TEMPORARY PID
                                            case 0:
                                                # PID TUNER GOTO
                                                self.main.processor._pidf_alt_vpa.set(kp=msg.param1, ti=msg.param2, td=msg.param3)
                                                self.main.processor.spf_altitude = msg.param4
                                                logging.info(f"New PID state: {msg.param1}, {msg.param2}, {msg.param3} @ {msg.param4}")
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
                                            # Command long only
                                            case m.MAV_CMD_DO_SET_MODE | m.MAV_CMD_DO_CHANGE_ALTITUDE | m.MAV_CMD_DO_CHANGE_SPEED:
                                                self._mav_conn_gcs.mav.command_ack_send(msg.command, m.MAV_RESULT_COMMAND_LONG_ONLY, 255, 0, 0, 0)
                                            # Command unsupported
                                            case _:
                                                self._mav_conn_gcs.mav.command_ack_send(msg.command, m.MAV_RESULT_UNSUPPORTED, 255, 0, 0, 0)

                    try:
                        await asyncio.sleep(0)
                    except asyncio.exceptions.CancelledError:
                        self.main.stop.set()
                        raise
                except Exception as e:
                    logging.error(f'Error in Controller (RX): {e}')
        except KeyboardInterrupt:
            self.main.stop.set()
            raise
        finally:
            logging.debug('Closing Controller (RX)')

    async def _tx(self) -> None:
        """Transmit continuous messages."""
        logging.debug('Starting Controller (TX)')

        try:
            while not self.main.stop.is_set():
                try:
                    pass # self._mav_conn_gcs.msg_send(None)

                    try:
                        await asyncio.sleep(1 / self._txfreq)
                    except asyncio.exceptions.CancelledError:
                        self.main.stop.set()
                        raise
                except Exception as e:
                    logging.error(f'Error in Controller (TX): {e}')
        except KeyboardInterrupt:
            self.main.stop.set()
            raise
        finally:
            logging.debug('Closing Controller (TX)')

    async def _heartbeat(self) -> None:
        """Periodically publish a heartbeat message."""
        logging.debug('Starting Controller (Heartbeat)')

        try:
            while not self.main.stop.is_set():
                try:
                    self._mav_conn_gcs.mav.heartbeat_send(
                        m.MAV_TYPE_VTOL_RESERVED4, # 24
                        m.MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY,
                        int(self.main.state.mode),
                        int(self.main.state.custom_mode),
                        int(self.main.state.state)
                    )
                    
                    logging.debug('TX Heartbeat')

                    try:
                        await asyncio.sleep(1 / self._heartbeatfreq)
                    except asyncio.exceptions.CancelledError:
                        self.main.stop.set()
                        raise
                except Exception as e:
                    logging.error(f'Error in Controller (Heartbeat): {e}')
        except KeyboardInterrupt:
            self.main.stop.set()
            raise
        finally:
            logging.debug('Closing Controller (Heartbeat)')

    def close(self) -> None:
        """Close the instance."""
        self._mav_conn_gcs.close()
        logging.info('Closing Controller')


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

    def __init__(self, systemid: int = 1, config: str = './common/CONFIG.ini') -> None:
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

        assert isinstance(systemid, int) and systemid > 0 and systemid.bit_length() <= 8, 'System ID must be UINT8'

        self.systemid = systemid

        self.config = ConfigParser()
        self.config.read(config)

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

        import common.grapher as grapher

        self._grapher = grapher.Grapher(deque_len=300)

        try:
            while not self.stop.is_set():
                try:
                    self._grapher.add(eval(name))
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

    async def run(self, graph: str = None) -> None:
        """Run the main system components.

        Leaving graph set to default will not start a graphing window.

        Parameters
        ----------
        graph : str, optional
            The name of the variable to graph (default is None).
        """

        self.controller = Controller(self)
        self.io = MainIO(self)
        self.processor = Processor(self)
        self.navigator = Navigator(self)

        controller_manager = asyncio.create_task(self.controller.manager())
        await asyncio.sleep(0)

        logging.warning(f'Creating instance #{self.systemid}, waiting for boot command from GCS')

        try:
            await self.boot.wait()
        except asyncio.exceptions.CancelledError:
            controller_manager.cancel()
            await asyncio.sleep(0)
            logging.warning(f'Never booted, closing instance #{self.systemid}')
            quit()

        logging.warning(f'Booting instance #{self.systemid}...')

        boot_tasks = [
            asyncio.create_task(self.processor.boot()),
            asyncio.create_task(self.io.boot()),
        ]

        try:
            await asyncio.gather(*boot_tasks)
        except asyncio.exceptions.CancelledError:
            controller_manager.cancel()
            await asyncio.sleep(0)
            logging.warning(f'Ctrl-C during boot cycle, closing instance #{self.systemid}')
            quit()

        logging.warning(f'Boot successful on #{self.systemid}')

        for _ in range(5):
            self.state.inc_mode()

        tasks = [
            asyncio.create_task(self.processor.run()),
            asyncio.create_task(self.navigator.run()),
            asyncio.create_task(self.io.run()),
        ]

        if graph is not None:
            logging.info('Grapher on')
            tasks.append(asyncio.create_task(self._graph(name=graph)))

        try:
            await asyncio.gather(*tasks)
        except asyncio.exceptions.CancelledError:
            self.stop.set()

        logging.warning(f'Closing instance #{self.systemid}')

if __name__ == '__main__':
    import random
    main = Main(20)#random.randint(1, 255))
    asyncio.run(main.run(graph='main.rxdata.alt.altitude'))
