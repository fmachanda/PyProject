import asyncio
import calendar
import datetime
import logging
import math
import os
import shutil
import socket
import struct
import sys
import time
from configparser import ConfigParser
from functools import wraps
# import numpy as np

if os.path.basename(os.getcwd()) == 'uav':
    os.chdir('..')
sys.path.append(os.getcwd())
os.environ['CYPHAL_PATH']='./common/data_types/custom_data_types;./common/data_types/public_regulated_data_types'
os.environ['PYCYPHAL_PATH']='./common/pycyphal_generated'
os.environ['UAVCAN__DIAGNOSTIC__SEVERITY'] = '2'
os.environ['MAVLINK20'] = '1'

import pycyphal
import pycyphal.application
import uavcan
import uavcan_archived
from pymavlink import mavutil
from uavcan_archived.equipment import actuator, ahrs, air_data, esc, gnss, range_sensor

import common.find_xp as find_xp

m = mavutil.mavlink

config = ConfigParser()
config.read('./common/CONFIG.ini')

filehandler = logging.FileHandler('uav/xpio.log', mode='w')
filehandler.setLevel(logging.WARNING)
logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.INFO, handlers=[filehandler, logging.StreamHandler()])
os.system('cls' if os.name == 'nt' else 'clear')

stop = asyncio.Event()

rx_data = {
    b'fmuas/att/attitude_quaternion_x': 0.0, # 0
    b'fmuas/att/attitude_quaternion_y': 0.0,
    b'fmuas/att/attitude_quaternion_z': 0.0,
    b'fmuas/att/attitude_quaternion_w': 0.0,
    b'fmuas/att/rollrate': 0.0,
    b'fmuas/att/pitchrate': 0.0,
    b'fmuas/att/yawrate': 0.0,
    b'fmuas/att/an': 0.0,
    b'fmuas/att/ae': 0.0,
    b'fmuas/att/ad': 0.0,

    b'fmuas/gps/latitude': 0.0,
    b'fmuas/gps/longitude': 0.0,
    b'fmuas/gps/altitude': 0.0,
    b'fmuas/gps/vn': 0.0,
    b'fmuas/gps/ve': 0.0,
    b'fmuas/gps/vd': 0.0,

    b'fmuas/radalt/altitude': 0.0,

    b'fmuas/adc/ias': 0.0,
    b'fmuas/adc/aoa': 0.0,
    
    b'sim/time/paused': 1.0,
    b'sim/time/local_date_days': 0.0,
    b'sim/time/zulu_time_sec': 0.0,
    b'fmuas/clock/time': 0.0,
}

tx_data = [
    [b'fmuas/afcs/output/elevon1', 0.0],
    [b'fmuas/afcs/output/elevon2', 0.0],
    [b'fmuas/afcs/output/wing_tilt', 0.0],
    [b'fmuas/afcs/output/wing_stow', 0.0],
    [b'fmuas/afcs/output/throttle1', 0.0],
    [b'fmuas/afcs/output/throttle2', 0.0],
    [b'fmuas/afcs/output/throttle3', 0.0],
    [b'fmuas/afcs/output/throttle4', 0.0],
]

XP_FIND_TIMEOUT = 1
TX_DATA_LAST_SERVO = 3 # Index of final servo dref before esc
XP_FREQ = 60
FREQ = 60
FT_TO_M = 3.048e-1
KT_TO_MS = 5.14444e-1

type LoopedClass = XPConnect | TestXPConnect | ServoIO | ESCIO | AltitudeSensor | AttitudeSensor | GPSSensor | IASSensor | AOASensor | Clock | Camera | TestCamera


def async_loop_decorator(close=True):
    """Provide decorator to gracefully loop coroutines."""
    def decorator(func):
        @wraps(func) # Preserve metadata like func.__name__
        async def wrapper(self: LoopedClass, *args, **kwargs):
            try:
                while not stop.is_set():
                    try:
                        await func(self, *args, **kwargs)
                        await asyncio.sleep(0)
                    except asyncio.exceptions.CancelledError:
                        stop.set()
                        raise
                    except Exception as e:
                        logging.error(f"Error in {func.__name__}: {e}")
                        raise e
            except KeyboardInterrupt:
                stop.set()
                raise
            finally:
                if close:
                    await self.close()
                else:
                    logging.debug(f"Closing {func.__name__}")
        return wrapper
    return decorator


class XPConnect:
    def __init__(self, freq: int = XP_FREQ) -> None:
        self._freq = freq

        logging.info("Looking for X-Plane...")
        try:
            beacon=find_xp.find_xp(wait=XP_FIND_TIMEOUT)
        except find_xp.XPlaneIpNotFound:
            logging.error(f"X-Plane not found, switching to test mode...")
            raise
        self.X_PLANE_IP=beacon['ip']
        self.UDP_PORT=beacon['port']
        logging.warning("X-Plane found at IP: %s, port: %s" % (self.X_PLANE_IP,self.UDP_PORT))

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.X_PLANE_IP, 0))
    
        msg = struct.pack('<4sxf500s', b'DREF', 1.0, b'fmuas/python_running')
        self.sock.sendto(msg, (self.X_PLANE_IP, self.UDP_PORT))

        for index, dref in enumerate(rx_data.keys()):
            msg = struct.pack('<4sxii400s', b'RREF', self._freq, index, dref)
            self.sock.sendto(msg, (self.X_PLANE_IP, self.UDP_PORT))

    @async_loop_decorator()
    async def _xpconnect_run_loop(self) -> None:
        global rx_data

        data, _ = self.sock.recvfrom(2048)
        header = data[0:4]

        if header == b'RREF':
            num_values = int(len(data[5:]) / 8)
            for i in range(num_values):
                dref_info = data[(5 + 8 * i):(5 + 8 * (i + 1))]
                (index, value) = struct.unpack('<if', dref_info)
                if index < len(rx_data):
                    rx_data[list(rx_data.keys())[index]] = value

        for index, dref in enumerate(tx_data):
                msg = struct.pack('<4sxf500s', b'DREF', dref[1], dref[0])
                self.sock.sendto(msg, (self.X_PLANE_IP, self.UDP_PORT))

        await asyncio.sleep(1 / self._freq)

    async def run(self) -> None:
        logging.warning("Data streaming...\n----- Ctrl-C to exit -----")
        await self._xpconnect_run_loop()
        
    async def close(self) -> None:
        logging.debug("Closing XPL")

        for index, dref in enumerate(rx_data.keys()):
            msg = struct.pack('<4sxii400s', b'RREF', 0, index, dref)
            self.sock.sendto(msg, (self.X_PLANE_IP, self.UDP_PORT))
        logging.info("Stopped listening for drefs")

        # line1 = "PYTHON CONNECTION LOST"
        # line2 = ""
        # line3 = "Restart python script to unpause."
        # line4 = "See FMUAS documentation for details."
        # msg = struct.pack('<4sx240s240s240s240s', b'ALRT',
        #                 line1.encode('utf-8'),
        #                 line2.encode('utf-8'),
        #                 line3.encode('utf-8'),
        #                 line4.encode('utf-8'))
        # self.sock.sendto(msg, (self.X_PLANE_IP, self.UDP_PORT))
        
        msg = struct.pack('<4sxf500s', b'DREF', 0.0, b'fmuas/python_running')
        self.sock.sendto(msg, (self.X_PLANE_IP, self.UDP_PORT))
        logging.info("LUA suspended")


class TestXPConnect:
    def __init__(self, rx_indices: list = None, freq: int = XP_FREQ) -> None:
        self._boot_time = time.time_ns() // 1000 # Used for sinusoidal data
        self._time = 0
        self._freq = freq
        self.rx_indices = rx_indices

        logging.info("Looking for X-Plane...")
        self.X_PLANE_IP="TEST"
        self.UDP_PORT=0
        logging.warning("X-Plane found at IP: %s, port: %s" % (self.X_PLANE_IP,self.UDP_PORT))

    @async_loop_decorator()
    async def _testxpconnect_run_loop(self) -> None:
        global rx_data

        self._time = time.time_ns()//1000 - self._boot_time
        rx_data[b'fmuas/clock/time'] = time.time() - self._boot_time/1e6

        if self.rx_indices is not None:
            for i in self.rx_indices:
                rx_data[list(rx_data.keys())[i]] = 20*math.sin(self._time/2e6)

        now = datetime.datetime.now()
        rx_data[b'sim/time/zulu_time_sec'] = (now - now.replace(hour=0, minute=0, second=0, microsecond=0)).total_seconds()
        rx_data[b'sim/time/paused'] = 0

        await asyncio.sleep(1 / self._freq)

    async def run(self) -> None:
        logging.warning("Data streaming...\n----- Ctrl-C to exit -----")
        await self._testxpconnect_run_loop()

    async def close(self) -> None:
        logging.debug("Closing XPL")
        logging.info("Stopped listening for drefs")
        logging.info("LUA suspended")

#region nodes
class ServoIO:
    def __init__(self, freq: int = FREQ) -> None:
        self._registry = pycyphal.application.make_registry(environment_variables={
            'UAVCAN__NODE__ID'              :config.get('node_ids', 'servoio'),
            'UAVCAN__UDP__IFACE'            :config.get('main', 'udp'),

            'UAVCAN__SUB__SERVO__ID'        :config.get('subject_ids', 'servo'),

            'UAVCAN__PUB__SERVO1_STATUS__ID':config.get('subject_ids', 'servo1_status'),
            'UAVCAN__PUB__SERVO2_STATUS__ID':config.get('subject_ids', 'servo2_status'),
            'UAVCAN__PUB__SERVO3_STATUS__ID':config.get('subject_ids', 'servo3_status'),
            'UAVCAN__PUB__SERVO4_STATUS__ID':config.get('subject_ids', 'servo4_status'),
        })

        self._freq = freq
        
        node_info = uavcan.node.GetInfo_1.Response(
            software_version=uavcan.node.Version_1(major=1, minor=0),
            name='fmuas.xpinterface.servoio',
        )

        self._node = pycyphal.application.make_node(node_info, self._registry)

        self._node.heartbeat_publisher.mode = uavcan.node.Mode_1.OPERATIONAL
        self._node.heartbeat_publisher.vendor_specific_status_code = os.getpid() % 100
        
        self._sub_servo = self._node.make_subscriber(actuator.ArrayCommand_1, 'servo')
        
        self._pub_servo1_status = self._node.make_publisher(actuator.Status_1, 'servo1_status')
        self._pub_servo2_status = self._node.make_publisher(actuator.Status_1, 'servo2_status')
        self._pub_servo3_status = self._node.make_publisher(actuator.Status_1, 'servo3_status')
        self._pub_servo4_status = self._node.make_publisher(actuator.Status_1, 'servo4_status')

        self._node.start()

    @async_loop_decorator()
    async def _servoio_run_loop(self) -> None:
        await self._pub_servo1_status.publish(actuator.Status_1(
            0, # id
            float('nan'), # position
            float('nan'), # force
            float('nan'), # speed
            actuator.Status_1.POWER_RATING_PCT_UNKNOWN
        ))
        
        await self._pub_servo2_status.publish(actuator.Status_1(
            1, # id
            float('nan'), # position
            float('nan'), # force
            float('nan'), # speed
            actuator.Status_1.POWER_RATING_PCT_UNKNOWN
        ))

        await self._pub_servo3_status.publish(actuator.Status_1(
            2, # id
            float('nan'), # position
            float('nan'), # force
            float('nan'), # speed
            actuator.Status_1.POWER_RATING_PCT_UNKNOWN
        ))

        await self._pub_servo4_status.publish(actuator.Status_1(
            3, # id
            float('nan'), # position
            float('nan'), # force
            float('nan'), # speed
            actuator.Status_1.POWER_RATING_PCT_UNKNOWN
        ))

        await asyncio.sleep(1 / self._freq)

    async def run(self) -> None:
        """docstring placeholder"""
        def on_servo(msg: actuator.ArrayCommand_1, _: pycyphal.transport.TransferFrom) -> None:
            for index, command in enumerate(msg.commands[:TX_DATA_LAST_SERVO]):
                if command.command_type==actuator.Command_1.COMMAND_TYPE_POSITION:
                    tx_data[index][1] = math.degrees(command.command_value)
        self._sub_servo.receive_in_background(on_servo)

        await self._servoio_run_loop()

    async def close(self) -> None:
        logging.debug("Closing SRV")
        self._node.close()


class ESCIO:
    def __init__(self, freq: int = FREQ) -> None:
        self._registry = pycyphal.application.make_registry(environment_variables={
            'UAVCAN__NODE__ID'              :config.get('node_ids', 'escio'),
            'UAVCAN__UDP__IFACE'            :config.get('main', 'udp'),

            'UAVCAN__SUB__ESC__ID'          :config.get('subject_ids', 'esc'),

            'UAVCAN__PUB__ESC1_STATUS__ID'  :config.get('subject_ids', 'esc1_status'),
            'UAVCAN__PUB__ESC2_STATUS__ID'  :config.get('subject_ids', 'esc2_status'),
            'UAVCAN__PUB__ESC3_STATUS__ID'  :config.get('subject_ids', 'esc3_status'),
            'UAVCAN__PUB__ESC4_STATUS__ID'  :config.get('subject_ids', 'esc4_status'),
        })

        self._freq = freq
        
        node_info = uavcan.node.GetInfo_1.Response(
            software_version=uavcan.node.Version_1(major=1, minor=0),
            name='fmuas.xpinterface.escio',
        )

        self._node = pycyphal.application.make_node(node_info, self._registry)

        self._node.heartbeat_publisher.mode = uavcan.node.Mode_1.OPERATIONAL
        self._node.heartbeat_publisher.vendor_specific_status_code = os.getpid() % 100
        
        self._sub_esc = self._node.make_subscriber(esc.RawCommand_1, 'esc')

        self._pub_esc1_status = self._node.make_publisher(esc.Status_1, 'esc1_status')
        self._pub_esc2_status = self._node.make_publisher(esc.Status_1, 'esc2_status')
        self._pub_esc3_status = self._node.make_publisher(esc.Status_1, 'esc3_status')
        self._pub_esc4_status = self._node.make_publisher(esc.Status_1, 'esc4_status')

        self._node.start()

    @async_loop_decorator()
    async def _escio_run_loop(self) -> None:
        await self._pub_esc1_status.publish(esc.Status_1(
            0, # error count
            float('nan'), # voltage
            float('nan'), # current
            float('nan'), # temperature
            0, # RPM (int)
            0, # power rating pct (int)
            0 # esc_index
        ))
        
        await self._pub_esc2_status.publish(esc.Status_1(
            0, # error count
            float('nan'), # voltage
            float('nan'), # current
            float('nan'), # temperature
            0, # RPM (int)
            0, # power rating pct (int)
            1 # esc_index
        ))

        await self._pub_esc3_status.publish(esc.Status_1(
            0, # error count
            float('nan'), # voltage
            float('nan'), # current
            float('nan'), # temperature
            0, # RPM (int)
            0, # power rating pct (int)
            2 # esc_index
        ))

        await self._pub_esc4_status.publish(esc.Status_1(
            0, # error count
            float('nan'), # voltage
            float('nan'), # current
            float('nan'), # temperature
            0, # RPM (int)
            0, # power rating pct (int)
            3 # esc_index
        ))

        await asyncio.sleep(1 / self._freq)

    async def run(self) -> None:
        """docstring placeholder"""
        def on_esc(msg: esc.RawCommand_1, _: pycyphal.transport.TransferFrom) -> None:
            for index, value in enumerate(msg.cmd[(TX_DATA_LAST_SERVO+1):]):
                tx_data[index + 1 + TX_DATA_LAST_SERVO][1] = value / 8192
        self._sub_esc.receive_in_background(on_esc)

        await self._escio_run_loop()

    async def close(self) -> None:
        logging.debug("Closing ESC")
        self._node.close()


class AttitudeSensor:
    def __init__(self, freq: int = FREQ) -> None:
        self._registry = pycyphal.application.make_registry(environment_variables={
            'UAVCAN__NODE__ID'                      :config.get('node_ids', 'attitudesensor'),
            'UAVCAN__UDP__IFACE'                    :config.get('main', 'udp'),

            'UAVCAN__PUB__ATTITUDE__ID'             :config.get('subject_ids', 'attitude'),

            'UAVCAN__SUB__CLOCK_SYNC_TIME__ID'      :config.get('subject_ids', 'clock_sync_time'),
            # 'UAVCAN__SUB__CLOCK_SYNC_TIME_LAST__ID' :config.get('subject_ids', 'clock_sync_time_last'),

            # 'UAVCAN__SUB__GPS_SYNC_TIME__ID'        :config.get('subject_ids', 'gps_sync_time'),
            # 'UAVCAN__SUB__GPS_SYNC_TIME_LAST__ID'   :config.get('subject_ids', 'gps_sync_time_last'),

            # 'UAVCAN__CLN__CLOCK_SYNC_INFO__ID'      :config.get('service_ids', 'clock_sync_info'),
            # 'UAVCAN__CLN__GPS_SYNC_INFO__ID'        :config.get('service_ids', 'gps_sync_info'),
        })
        
        self._freq = freq
        self._time = 0.0
        
        node_info = uavcan.node.GetInfo_1.Response(
            software_version=uavcan.node.Version_1(major=1, minor=0),
            name='fmuas.xpinterface.attitudesensor',
        )

        self._node = pycyphal.application.make_node(node_info, self._registry)

        self._node.heartbeat_publisher.mode = uavcan.node.Mode_1.OPERATIONAL
        self._node.heartbeat_publisher.vendor_specific_status_code = os.getpid() % 100
        
        self._pub_att = self._node.make_publisher(ahrs.Solution_1, 'attitude')

        self._sub_clock_sync_time = self._node.make_subscriber(uavcan.time.SynchronizedTimestamp_1, 'clock_sync_time')
        # self._sub_clock_sync_time_last = self._node.make_subscriber(uavcan.time.Synchronization_1, 'clock_sync_time_last')

        # self._sub_gps_sync_time = self._node.make_subscriber(uavcan.time.SynchronizedTimestamp_1, 'gps_sync_time')
        # self._sub_gps_sync_time_last = self._node.make_subscriber(uavcan.time.Synchronization_1, 'gps_sync_time_last')

        # self._cln_clock_sync_info = self._node.make_client(uavcan.time.GetSynchronizationMasterInfo_0, config.getint('service_ids', 'clock_sync_info'), 'clock_sync_info')
        # self._cln_gps_sync_info = self._node.make_client(uavcan.time.GetSynchronizationMasterInfo_0, config.getint('service_ids', 'gps_sync_info'), 'gps_sync_info')

        self._node.start()

    @async_loop_decorator()
    async def _attitudesensor_run_loop(self) -> None:
        m = ahrs.Solution_1(
            uavcan_archived.Timestamp_1(self._time),
            [
                rx_data[b'fmuas/att/attitude_quaternion_x'], 
                rx_data[b'fmuas/att/attitude_quaternion_y'], 
                rx_data[b'fmuas/att/attitude_quaternion_z'], 
                rx_data[b'fmuas/att/attitude_quaternion_w']
            ],
            [float('nan')],
            [
                rx_data[b'fmuas/att/rollrate'],
                rx_data[b'fmuas/att/pitchrate'],
                rx_data[b'fmuas/att/yawrate'],
            ],
            [float('nan')],
            [
                rx_data[b'fmuas/att/an'],
                rx_data[b'fmuas/att/ae'],
                rx_data[b'fmuas/att/ad'],
            ],
            [float('nan')]
        )
        await self._pub_att.publish(m)
        await asyncio.sleep(1 / self._freq)

    async def run(self) -> None:
        """docstring placeholder"""
        def on_time(msg: uavcan.time.SynchronizedTimestamp_1, _: pycyphal.transport.TransferFrom) -> None:
            self._time = msg.microsecond
        self._sub_clock_sync_time.receive_in_background(on_time)

        await self._attitudesensor_run_loop()

    async def close(self) -> None:
        logging.debug("Closing ATT")
        self._node.close()


class AltitudeSensor:
    def __init__(self, freq: int = FREQ) -> None:
        self._registry = pycyphal.application.make_registry(environment_variables={
            'UAVCAN__NODE__ID'              :config.get('node_ids', 'altitudesensor'),
            'UAVCAN__UDP__IFACE'            :config.get('main', 'udp'),

            'UAVCAN__PUB__ALTITUDE__ID'     :config.get('subject_ids', 'altitude'),

            'UAVCAN__SUB__CLOCK_SYNC_TIME__ID'      :config.get('subject_ids', 'clock_sync_time'),
            # 'UAVCAN__SUB__CLOCK_SYNC_TIME_LAST__ID' :config.get('subject_ids', 'clock_sync_time_last'),

            # 'UAVCAN__SUB__GPS_SYNC_TIME__ID'        :config.get('subject_ids', 'gps_sync_time'),
            # 'UAVCAN__SUB__GPS_SYNC_TIME_LAST__ID'   :config.get('subject_ids', 'gps_sync_time_last'),

            # 'UAVCAN__CLN__CLOCK_SYNC_INFO__ID'      :config.get('service_ids', 'clock_sync_info'),
            # 'UAVCAN__CLN__GPS_SYNC_INFO__ID'        :config.get('service_ids', 'gps_sync_info'),
        })

        self._freq = freq
        self._time = 0.0
        
        node_info = uavcan.node.GetInfo_1.Response(
            software_version=uavcan.node.Version_1(major=1, minor=0),
            name='fmuas.xpinterface.altitudesensor',
        )

        self._node = pycyphal.application.make_node(node_info, self._registry)

        self._node.heartbeat_publisher.mode = uavcan.node.Mode_1.OPERATIONAL
        self._node.heartbeat_publisher.vendor_specific_status_code = os.getpid() % 100
        
        self._pub_alt = self._node.make_publisher(range_sensor.Measurement_1, 'altitude')

        self._sub_clock_sync_time = self._node.make_subscriber(uavcan.time.SynchronizedTimestamp_1, 'clock_sync_time')
        # self._sub_clock_sync_time_last = self._node.make_subscriber(uavcan.time.Synchronization_1, 'clock_sync_time_last')

        # self._sub_gps_sync_time = self._node.make_subscriber(uavcan.time.SynchronizedTimestamp_1, 'gps_sync_time')
        # self._sub_gps_sync_time_last = self._node.make_subscriber(uavcan.time.Synchronization_1, 'gps_sync_time_last')

        # self._cln_clock_sync_info = self._node.make_client(uavcan.time.GetSynchronizationMasterInfo_0, config.getint('service_ids', 'clock_sync_info'), 'clock_sync_info')
        # self._cln_gps_sync_info = self._node.make_client(uavcan.time.GetSynchronizationMasterInfo_0, config.getint('service_ids', 'gps_sync_info'), 'gps_sync_info')

        self._node.start()
    
    @async_loop_decorator()
    async def _altitudesensor_run_loop(self) -> None:
        m = range_sensor.Measurement_1(
            uavcan_archived.Timestamp_1(self._time),
            0, # Sensor ID
            uavcan_archived.CoarseOrientation_1([0.0,0.0,0.0], True), # TODO
            1.5, # FOV
            range_sensor.Measurement_1.SENSOR_TYPE_RADAR,
            range_sensor.Measurement_1.READING_TYPE_VALID_RANGE,
            rx_data[b'fmuas/radalt/altitude']
        )
        await self._pub_alt.publish(m)
        await asyncio.sleep(1 / self._freq)

    async def run(self) -> None:
        """docstring placeholder"""
        def on_time(msg: uavcan.time.SynchronizedTimestamp_1, _: pycyphal.transport.TransferFrom) -> None:
            self._time = msg.microsecond
        self._sub_clock_sync_time.receive_in_background(on_time)
        
        await self._altitudesensor_run_loop()

    async def close(self) -> None:
        logging.debug("Closing ALT")
        self._node.close()


class GPSSensor:
    def __init__(self, freq: int = FREQ) -> None:
        self._registry = pycyphal.application.make_registry(environment_variables={
            'UAVCAN__NODE__ID'                      :config.get('node_ids', 'gpssensor'),
            'UAVCAN__UDP__IFACE'                    :config.get('main', 'udp'),

            'UAVCAN__PUB__GPS__ID'                  :config.get('subject_ids', 'gps'),

            'UAVCAN__PUB__GPS_SYNC_TIME__ID'        :config.get('subject_ids', 'gps_sync_time'),
            'UAVCAN__PUB__GPS_SYNC_TIME_LAST__ID'   :config.get('subject_ids', 'gps_sync_time_last'),

            'UAVCAN__SRV__GPS_SYNC_INFO__ID'        :config.get('service_ids', 'gps_sync_info'),

            'UAVCAN__SUB__CLOCK_SYNC_TIME__ID'      :config.get('subject_ids', 'clock_sync_time'),
            # 'UAVCAN__SUB__CLOCK_SYNC_TIME_LAST__ID' :config.get('subject_ids', 'clock_sync_time_last'),

            # 'UAVCAN__CLN__CLOCK_SYNC_INFO__ID'      :config.get('service_ids', 'clock_sync_info'),
        })

        self._freq = freq
        self._time = 0.0
        
        node_info = uavcan.node.GetInfo_1.Response(
            software_version=uavcan.node.Version_1(major=1, minor=0),
            name='fmuas.xpinterface.gpssensor',
        )

        self._node = pycyphal.application.make_node(node_info, self._registry)

        self._node.heartbeat_publisher.mode = uavcan.node.Mode_1.OPERATIONAL
        self._node.heartbeat_publisher.vendor_specific_status_code = os.getpid() % 100
        
        self._pub_gps = self._node.make_publisher(gnss.Fix2_1, 'gps')
        
        self._srv_sync_info = self._node.get_server(uavcan.time.GetSynchronizationMasterInfo_0, 'gps_sync_info')
        self._srv_sync_info.serve_in_background(self._serve_sync_master_info)
        
        self._pub_gps_sync_time = self._node.make_publisher(uavcan.time.SynchronizedTimestamp_1, 'gps_sync_time')
        self._pub_gps_sync_time_last = self._node.make_publisher(uavcan.time.Synchronization_1, 'gps_sync_time')

        self._sub_clock_sync_time = self._node.make_subscriber(uavcan.time.SynchronizedTimestamp_1, 'clock_sync_time')
        # self._sub_clock_sync_time_last = self._node.make_subscriber(uavcan.time.Synchronization_1, 'clock_sync_time_last')

        # self._cln_clock_sync_info = self._node.make_client(uavcan.time.GetSynchronizationMasterInfo_0, config.getint('service_ids', 'clock_sync_info'), 'clock_sync_info')

        self._node.start()

    @staticmethod
    async def _serve_sync_master_info(
            request: uavcan.time.GetSynchronizationMasterInfo_0.Request, 
            metadata: pycyphal.presentation.ServiceRequestMetadata,) -> uavcan.time.GetSynchronizationMasterInfo_0.Response:
        logging.info("Execute command request %s from node %d", request, metadata.client_node_id)
        return uavcan.time.GetSynchronizationMasterInfo_0.Response(
            0.0, # error_variance
            uavcan.time.TimeSystem_0(uavcan.time.TimeSystem_0.TAI),
            uavcan.time.TAIInfo_0(uavcan.time.TAIInfo_0.DIFFERENCE_TAI_MINUS_GPS)
        )
    
    @async_loop_decorator()
    async def _gpssensor_run_loop(self) -> None:
        gnss_time = int(1e6*(calendar.timegm(datetime.datetime.strptime(str(datetime.date.today().year), '%Y').timetuple())
                             + (1+rx_data[b'sim/time/local_date_days'])*86400
                             + rx_data[b'sim/time/zulu_time_sec']))

        m = gnss.Fix2_1(
            uavcan_archived.Timestamp_1(self._time),
            uavcan_archived.Timestamp_1(gnss_time),
            gnss.Fix2_1.GNSS_TIME_STANDARD_GPS,
            gnss.Fix2_1.NUM_LEAP_SECONDS_UNKNOWN, # TODO
            int(rx_data[b'fmuas/gps/longitude']),
            int(rx_data[b'fmuas/gps/latitude']),
            0, # Ellipsoid
            int(rx_data[b'fmuas/gps/altitude']),
            [rx_data[b'fmuas/gps/vn'], rx_data[b'fmuas/gps/ve'], rx_data[b'fmuas/gps/vd']],
            5, # Sats used
            gnss.Fix2_1.STATUS_3D_FIX,
            gnss.Fix2_1.MODE_SINGLE,
            0, # Submode
            [float('nan')],
            3.5,
            gnss.ECEFPositionVelocity_1([float('nan'), float('nan'), float('nan')], [0, 0, 0], [float('nan')])
        )
        await self._pub_gps.publish(m)
        await asyncio.sleep(1 / self._freq)

    async def run(self) -> None:
        """docstring placeholder"""
        def on_time(msg: uavcan.time.SynchronizedTimestamp_1, _: pycyphal.transport.TransferFrom) -> None:
            self._time = msg.microsecond
        self._sub_clock_sync_time.receive_in_background(on_time)

        await self._gpssensor_run_loop()

    async def close(self) -> None:
        logging.debug("Closing GPS")
        self._node.close()


class IASSensor:
    def __init__(self, freq: int = FREQ) -> None:
        self._registry = pycyphal.application.make_registry(environment_variables={
            'UAVCAN__NODE__ID'              :config.get('node_ids', 'iassensor'),
            'UAVCAN__UDP__IFACE'            :config.get('main', 'udp'),

            'UAVCAN__PUB__IAS__ID'     :config.get('subject_ids', 'ias'),
        })

        self._freq = freq
        
        node_info = uavcan.node.GetInfo_1.Response(
            software_version=uavcan.node.Version_1(major=1, minor=0),
            name='fmuas.xpinterface.iassensor',
        )

        self._node = pycyphal.application.make_node(node_info, self._registry)

        self._node.heartbeat_publisher.mode = uavcan.node.Mode_1.OPERATIONAL
        self._node.heartbeat_publisher.vendor_specific_status_code = os.getpid() % 100
        
        self._pub_ias = self._node.make_publisher(air_data.IndicatedAirspeed_1, 'ias')

        self._node.start()

    @async_loop_decorator()
    async def _iassensor_run_loop(self) -> None:
        m = air_data.IndicatedAirspeed_1(
            rx_data[b'fmuas/adc/ias'],
            float('nan')
        )
        await self._pub_ias.publish(m)
        await asyncio.sleep(1 / self._freq)

    async def run(self) -> None:
        await self._iassensor_run_loop()

    async def close(self) -> None:
        logging.debug("Closing IAS")
        self._node.close()


class AOASensor:
    def __init__(self, freq: int = FREQ) -> None:
        self._registry = pycyphal.application.make_registry(environment_variables={
            'UAVCAN__NODE__ID'              :config.get('node_ids', 'aoasensor'),
            'UAVCAN__UDP__IFACE'            :config.get('main', 'udp'),

            'UAVCAN__PUB__AOA__ID'     :config.get('subject_ids', 'aoa'),
        })

        self._freq = freq
        
        node_info = uavcan.node.GetInfo_1.Response(
            software_version=uavcan.node.Version_1(major=1, minor=0),
            name='fmuas.xpinterface.aoasensor',
        )

        self._node = pycyphal.application.make_node(node_info, self._registry)

        self._node.heartbeat_publisher.mode = uavcan.node.Mode_1.OPERATIONAL
        self._node.heartbeat_publisher.vendor_specific_status_code = os.getpid() % 100
        
        self._pub_aoa = self._node.make_publisher(air_data.AngleOfAttack_1, 'aoa')

        self._node.start()

    @async_loop_decorator()
    async def _aoasensor_run_loop(self) -> None:
        m = air_data.AngleOfAttack_1(
            air_data.AngleOfAttack_1.SENSOR_ID_LEFT,
            rx_data[b'fmuas/adc/aoa'],
            float('nan')
        )
        await self._pub_aoa.publish(m)
        await asyncio.sleep(1 / self._freq)

    async def run(self) -> None:
        await self._aoasensor_run_loop()

    async def close(self) -> None:
        logging.debug("Closing AOA")
        self._node.close()


class Clock:
    def __init__(self) -> None:
        self._registry = pycyphal.application.make_registry(environment_variables={
            'UAVCAN__NODE__ID'                      :config.get('node_ids', 'clock'),
            'UAVCAN__UDP__IFACE'                    :config.get('main', 'udp'),

            'UAVCAN__PUB__CLOCK_SYNC_TIME__ID'      :config.get('subject_ids', 'clock_sync_time'),
            'UAVCAN__PUB__CLOCK_SYNC_TIME_LAST__ID' :config.get('subject_ids', 'clock_sync_time_last'),

            'UAVCAN__SRV__CLOCK_SYNC_INFO__ID'      :config.get('service_ids', 'clock_sync_info'),
        })

        self._sync_time = 0.0
        self._last_xpsecs = 0.0
        self._sync_secs = 0.0
        self._last_time = time.time()
        self._last_real = 0.0
        
        node_info = uavcan.node.GetInfo_1.Response(
            software_version=uavcan.node.Version_1(major=1, minor=0),
            name='fmuas.xpinterface.clock',
        )

        self._node = pycyphal.application.make_node(node_info, self._registry)

        self._node.heartbeat_publisher.mode = uavcan.node.Mode_1.OPERATIONAL
        self._node.heartbeat_publisher.vendor_specific_status_code = os.getpid() % 100
        
        self._srv_sync_info = self._node.get_server(uavcan.time.GetSynchronizationMasterInfo_0, 'clock_sync_info')
        self._srv_sync_info.serve_in_background(self._serve_sync_master_info)
        
        self._pub_sync_time = self._node.make_publisher(uavcan.time.SynchronizedTimestamp_1, 'clock_sync_time')
        self._pub_sync_time_last = self._node.make_publisher(uavcan.time.Synchronization_1, 'clock_sync_time_last')

        self._node.start()

    @staticmethod
    async def _serve_sync_master_info(
            request: uavcan.time.GetSynchronizationMasterInfo_0.Request, 
            metadata: pycyphal.presentation.ServiceRequestMetadata,)-> uavcan.time.GetSynchronizationMasterInfo_0.Response:
        logging.info("Execute command request %s from node %d", request, metadata.client_node_id)
        return uavcan.time.GetSynchronizationMasterInfo_0.Response(
            0.0, # error_variance
            uavcan.time.TimeSystem_0(uavcan.time.TimeSystem_0.MONOTONIC_SINCE_BOOT),
            uavcan.time.TAIInfo_0(uavcan.time.TAIInfo_0.DIFFERENCE_TAI_MINUS_UTC_UNKNOWN)
        )

    @async_loop_decorator()
    async def _clock_run_loop(self) -> None:
        await self._pub_sync_time_last.publish(uavcan.time.Synchronization_1(int(self._sync_time))) # Last timestamp

        self._xpsecs = rx_data[b'fmuas/clock/time']

        if abs(self._xpsecs - self._last_xpsecs) > 1.0: # Time jump
            logging.debug("XP time jumped")
            self._sync_secs += 1e-6
        elif self._xpsecs == self._last_xpsecs and rx_data[b'sim/time/paused'] != 1.0: # No time but running
            self._sync_secs += time.time() - self._last_time
        elif rx_data[b'sim/time/paused'] != 1.0: # Normal and running
            self._last_real += self._xpsecs - self._last_xpsecs
            self._sync_secs = self._last_real

        self._last_time = time.time()
        self._sync_time = int(self._sync_secs*1e6)
        self._last_xpsecs = self._xpsecs

        await self._pub_sync_time.publish(uavcan.time.SynchronizedTimestamp_1(int(self._sync_time))) # Current timestamp
        await asyncio.sleep(0)

    async def run(self) -> None:
        await self._clock_run_loop()

    async def close(self) -> None:
        logging.debug("Closing CLK")
        self._node.close()
#endregion

class Camera:
    DELAY = 0.5

    def __init__(self, xpconnection: XPConnect, target_id: int = 1) -> None:
        assert isinstance(xpconnection, XPConnect), "Must pass an instance of XPConnect"
        self.sock = xpconnection.sock
        self.X_PLANE_IP = xpconnection.X_PLANE_IP
        self.UDP_PORT = xpconnection.UDP_PORT
        
        assert isinstance(target_id,int) and 0<target_id<256, "Camera target ID must be UINT8"
        self.target_id = target_id

        self.xp_path = config.get('xplane', 'xp_screenshot_path')
        self.destination_dir = './stored_images'
        if not os.path.exists(self.destination_dir):
           os.makedirs(self.destination_dir)
        
        self.camera_mav_conn: mavutil.mavfile = mavutil.mavlink_connection(config.get('mavlink', 'camera_uav_conn'), source_system=self.target_id, source_component=m.MAV_COMP_ID_CAMERA, input=True)

    @async_loop_decorator()
    async def _camera_run_loop(self) -> None:
        msg = self.camera_mav_conn.recv_msg()
        if msg is not None and msg.get_srcSystem()==self.target_id and msg.target_system==self.target_id and msg.target_component==m.MAV_COMP_ID_CAMERA and msg.get_type() == 'COMMAND_LONG':
            if msg.command == m.MAV_CMD_IMAGE_START_CAPTURE:
                cap = asyncio.create_task(self._capture_cycle(int(msg.param3), msg.param2))
                self.camera_mav_conn.mav.command_ack_send(m.MAV_CMD_IMAGE_START_CAPTURE, m.MAV_RESULT_ACCEPTED, 255, 0, self.target_id, 0)
            elif msg.command == m.MAV_CMD_IMAGE_STOP_CAPTURE:
                try:
                    cap.cancel()
                    self.camera_mav_conn.mav.command_ack_send(m.MAV_CMD_IMAGE_STOP_CAPTURE, m.MAV_RESULT_ACCEPTED, 255, 0, self.target_id, 0)
                except AttributeError:
                    pass
                    self.camera_mav_conn.mav.command_ack_send(m.MAV_CMD_IMAGE_STOP_CAPTURE, m.MAV_RESULT_DENIED, 255, 0, self.target_id, 0)
            else:
                self.camera_mav_conn.mav.command_ack_send(msg.command, m.MAV_RESULT_UNSUPPORTED, 255, 0, self.target_id, 0)

        await asyncio.sleep(0)  

    async def run(self) -> None:
        # asyncio.create_task(self._heartbeat())
        await self._camera_run_loop()

    async def _capture(self) -> None:
        previous_file_list = [f for f in os.listdir(self.xp_path) if os.path.isfile(os.path.join(self.xp_path, f))]

        logging.info("Commanding screenshot...")
        self.sock.sendto(struct.pack('<4sx400s', b'CMND', b'fmuas/commands/image_capture'), (self.X_PLANE_IP, self.UDP_PORT))

        await asyncio.sleep(Camera.DELAY)

        new_file_list = [f for f in os.listdir(self.xp_path) if os.path.isfile(os.path.join(self.xp_path, f))]
        file_diff = [x for x in new_file_list if x not in previous_file_list]
        previous_file_list = new_file_list

        if len(file_diff) != 0:
            logging.debug(f"Detected file_diff: {file_diff}")
            for f in file_diff:
                ready = False
                last_modified = os.stat(os.path.join(self.xp_path, f)).st_mtime
                while not ready:
                    await asyncio.sleep(Camera.DELAY)
                    new_last_modified = os.stat(os.path.join(self.xp_path, f)).st_mtime
                    ready = (last_modified == new_last_modified)
                    last_modified = new_last_modified
                    logging.debug('Shutil waiting...')
                logging.debug("Shutil moving")
                if not os.path.exists(self.destination_dir):
                    os.makedirs(self.destination_dir)
                file_path = os.path.join(self.xp_path, f)
                try:
                    shutil.move(file_path, self.destination_dir)
                    logging.info(f"Moving file {f} to {self.destination_dir}")
                except shutil.Error:
                    logging.error(f"Error moving file {f} to {self.destination_dir}")
                except PermissionError:
                    logging.error(f"Error moving file {f} to {self.destination_dir}")
                finally:
                    pass

    async def _capture_cycle(self, iterations: int, period: float) -> None:
        if iterations != 0:
            for _ in range(iterations):
                try:
                    tstart = time.time()
                    await asyncio.gather(asyncio.create_task(self._capture()))
                    await asyncio.sleep(max(0, period - (time.time()-tstart)))
                except asyncio.exceptions.CancelledError:
                    break
        else:
            while True:
                try:
                    await asyncio.gather(asyncio.create_task(self._capture()))
                    await asyncio.sleep(period)
                except asyncio.exceptions.CancelledError:
                    break

    @async_loop_decorator(close=False)
    async def _camera_heartbeat_loop(self) -> None:
        self.camera_mav_conn.mav.heartbeat_send(
            m.MAV_TYPE_CAMERA,
            m.MAV_AUTOPILOT_INVALID,
            m.MAV_MODE_PREFLIGHT,
            0,
            m.MAV_STATE_ACTIVE
        )

    async def _heartbeat(self) -> None:
        await self._camera_heartbeat_loop()

    async def close(self) -> None:
        logging.debug("Closing CAM")
        self.camera_mav_conn.close()


class TestCamera:
    def __init__(self, xpconnection: TestXPConnect, target_id: int = 1) -> None:
        assert isinstance(xpconnection, TestXPConnect), "Must pass an instance of TestXPConnect"

        assert isinstance(target_id,int) and 0<target_id<256, "Camera target ID must be UINT8"
        self.target_id = target_id

        self.xp_path = config.get('xplane', 'xp_screenshot_path')
        
        self.camera_mav_conn: mavutil.mavfile = mavutil.mavlink_connection(config.get('mavlink', 'camera_uav_conn'), source_system=self.target_id, source_component=m.MAV_COMP_ID_CAMERA, input=True)
        
    @async_loop_decorator()
    async def _testcamera_run_loop(self) -> None:
        msg = self.camera_mav_conn.recv_msg()
        if msg is not None and msg.get_srcSystem()==self.target_id and msg.target_system==self.target_id and msg.target_component==m.MAV_COMP_ID_CAMERA and msg.get_type() == 'COMMAND_LONG':
            if msg.command == m.MAV_CMD_IMAGE_START_CAPTURE:
                cap = asyncio.create_task(self._capture_cycle(int(msg.param3), msg.param2))
                self.camera_mav_conn.mav.command_ack_send(m.MAV_CMD_IMAGE_START_CAPTURE, m.MAV_RESULT_ACCEPTED, 255, 0, self.target_id, 0)
            elif msg.command == m.MAV_CMD_IMAGE_STOP_CAPTURE:
                try:
                    cap.cancel()
                    self.camera_mav_conn.mav.command_ack_send(m.MAV_CMD_IMAGE_STOP_CAPTURE, m.MAV_RESULT_ACCEPTED, 255, 0, self.target_id, 0)
                except AttributeError:
                    pass
                    self.camera_mav_conn.mav.command_ack_send(m.MAV_CMD_IMAGE_STOP_CAPTURE, m.MAV_RESULT_DENIED, 255, 0, self.target_id, 0)
            else:
                self.camera_mav_conn.mav.command_ack_send(msg.command, m.MAV_RESULT_UNSUPPORTED, 255, 0, self.target_id, 0)

        await asyncio.sleep(0)

    async def run(self) -> None:
        # asyncio.create_task(self._heartbeat())
        await self._testcamera_run_loop()

    async def _capture(self) -> None:
        logging.info("Commanding screenshot...")
        await asyncio.sleep(Camera.DELAY)

    async def _capture_cycle(self, iterations: int, period: float) -> None:
        if iterations != 0:
            for _ in range(iterations):
                try:
                    tstart = time.time()
                    await asyncio.gather(asyncio.create_task(self._capture()))
                    await asyncio.sleep(max(0, period - (time.time()-tstart)))
                except asyncio.exceptions.CancelledError:
                    break
        else:
            while True:
                try:
                    await asyncio.gather(asyncio.create_task(self._capture()))
                    await asyncio.sleep(period)
                except asyncio.exceptions.CancelledError:
                    break

    @async_loop_decorator(close=False)
    async def _testcamera_heartbeat_loop(self) -> None:
        self.camera_mav_conn.mav.heartbeat_send(
            m.MAV_TYPE_CAMERA,
            m.MAV_AUTOPILOT_INVALID,
            m.MAV_MODE_PREFLIGHT,
            0,
            m.MAV_STATE_ACTIVE
        )
        await asyncio.sleep(1)

    async def _heartbeat(self) -> None:
        await self._testcamera_heartbeat_loop()

    async def close(self) -> None:
        logging.debug("Closing CAM")
        self.camera_mav_conn.close()


async def main():
    try:
        xpl = XPConnect()
        cam = Camera(xpl, target_id=config.getint('main', 'uav_id'))
    except find_xp.XPlaneIpNotFound or KeyboardInterrupt:
        xpl = TestXPConnect()
        cam = TestCamera(xpl, target_id=config.getint('main', 'uav_id'))
    clk = Clock()
    att = AttitudeSensor()
    alt = AltitudeSensor()
    gps = GPSSensor()
    ias = IASSensor()
    aoa = AOASensor()
    srv = ServoIO()
    esc = ESCIO()

    tasks = [
        asyncio.create_task(xpl.run()),
        asyncio.create_task(cam.run()),
        asyncio.create_task(clk.run()),
        asyncio.create_task(att.run()),
        asyncio.create_task(alt.run()),
        asyncio.create_task(gps.run()),
        asyncio.create_task(ias.run()),
        asyncio.create_task(aoa.run()),
        asyncio.create_task(srv.run()),
        asyncio.create_task(esc.run()),
    ]

    try:
        await asyncio.gather(*tasks)
    except asyncio.exceptions.CancelledError:
        stop.set()

    logging.warning("Nodes closed")


if __name__ == '__main__':
    asyncio.run(main())
