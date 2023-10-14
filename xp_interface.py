import os
import asyncio
import logging
import numpy as np
import math
import struct
import socket
import time
import datetime
import calendar
from configparser import ConfigParser
from pymavlink import mavutil
import shutil

os.environ['CYPHAL_PATH']='./data_types/custom_data_types;./data_types/public_regulated_data_types'
os.environ['PYCYPHAL_PATH']='./pycyphal_generated'
os.environ['UAVCAN__DIAGNOSTIC__SEVERITY'] = '2'

import pycyphal
import pycyphal.application
import uavcan_archived
from uavcan_archived.equipment import actuator, esc, ahrs, gnss, range_sensor, air_data
import uavcan

config = ConfigParser()
config.read('./config.ini')

logging.basicConfig(level=logging.INFO)
os.system('cls' if os.name == 'nt' else 'clear')

stop = asyncio.Event()

rx_data = [

        [b'fmuas/gpsins/roll', 0.0], # 0
        [b'fmuas/gpsins/pitch', 0.0],
        [b'fmuas/gpsins/yaw', 0.0],
        [b'fmuas/gpsins/rollrate', 0.0],
        [b'fmuas/gpsins/pitchrate', 0.0],
        [b'fmuas/gpsins/yawrate', 0.0], # 5
        [b'fmuas/radalt/altitude', 0.0],
        [b'fmuas/gpsins/latitude', 0.0],
        [b'fmuas/gpsins/longitude', 0.0],
        [b'fmuas/gpsins/xspeed', 0.0],
        [b'fmuas/gpsins/yspeed', 0.0], # 10
        [b'fmuas/adc/ias', 0.0],
        [b'fmuas/adc/aoa', 0.0],
        [b'fmuas/adc/slip', 0.0],
        [b'sim/time/paused', 1.0],
        [b'sim/operation/misc/frame_rate_period', 0.0], #15
        [b'sim/time/local_date_days', 0.0],
        [b'sim/time/zulu_time_sec', 0.0],
        [b'fmuas/clock/time', 0.0],

    ]

tx_data = [

        [b'fmuas/afcs/output/elevon1', 0.0],
        [b'fmuas/afcs/output/elevon2', 0.0],
        [b'fmuas/afcs/output/yaw', 0.0],
        [b'fmuas/afcs/output/wing_tilt', 0.0],
        [b'fmuas/afcs/output/throttle1', 0.0],
        [b'fmuas/afcs/output/throttle2', 0.0],
        [b'fmuas/afcs/output/throttle3', 0.0],
        [b'fmuas/afcs/output/throttle4', 0.0],

    ]

TX_DATA_LAST_SERVO = 3 # Index of final servo dref before esc

XP_FREQ = 60
FREQ = 60

FT_TO_M = 3.048e-1
KT_TO_MS = 5.14444e-1

class XPConnect:

    def __init__(self, freq:int=XP_FREQ) -> None:

        import ext.find_xp as find_xp

        self._freq = freq

        logging.info("Looking for X-Plane...")
        try:
            beacon=find_xp.find_xp(wait=0)
        except KeyboardInterrupt:
            quit()
        self.X_PLANE_IP=beacon['ip']
        self.UDP_PORT=beacon['port']
        logging.warning("X-Plane found at IP: %s, port: %s" % (self.X_PLANE_IP,self.UDP_PORT))

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.X_PLANE_IP, 0))
    
        msg = struct.pack('<4sxf500s', b'DREF', 1.0, b'fmuas/python_running')
        self.sock.sendto(msg, (self.X_PLANE_IP, self.UDP_PORT))

        for index, dref in enumerate(rx_data):
            msg = struct.pack("<4sxii400s", b'RREF', self._freq, index, dref[0])
            self.sock.sendto(msg, (self.X_PLANE_IP, self.UDP_PORT))

    async def run(self) -> None:

        global rx_data

        logging.warning("Data streaming...\n----- Ctrl-C to exit -----")

        while not stop.is_set():
            try:

                data, _ = self.sock.recvfrom(2048)

                header = data[0:4]

                if header == b'RREF':

                    num_values = int(len(data[5:]) / 8)

                    for i in range(num_values):

                        dref_info = data[(5 + 8 * i):(5 + 8 * (i + 1))]
                        (index, value) = struct.unpack("<if", dref_info)
                        
                        if index < len(rx_data):
                            rx_data[index][1] = value

                for index, dref in enumerate(tx_data):
                        msg = struct.pack('<4sxf500s', b'DREF', dref[1], dref[0])
                        self.sock.sendto(msg, (self.X_PLANE_IP, self.UDP_PORT))

                try:
                    await asyncio.sleep(1 / self._freq)
                except asyncio.exceptions.CancelledError: 
                    stop.set()
                    raise

            except KeyboardInterrupt:
                stop.set()

            except Exception as e:
                logging.error("Socket error: ", e)
            
        self.close()
        
    def close(self) -> None:

        logging.debug('Closing XPL')

        for index, dref in enumerate(rx_data):
            msg = struct.pack("<4sxii400s", b'RREF', 0, index, dref[0])
            self.sock.sendto(msg, (self.X_PLANE_IP, self.UDP_PORT))
        logging.info('Stopped listening for drefs')

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
        logging.info('LUA suspended')

class TestXPConnect:

    def __init__(self, rx_indices:list=None, freq:int=XP_FREQ) -> None:

        self._boot_time = time.time_ns() // 1000 # Used for sinusoidal data
        self._time = 0
        self._freq = freq
        self.rx_indices = rx_indices

        logging.info("Looking for X-Plane...")
        self.X_PLANE_IP='TEST'
        self.UDP_PORT=0
        logging.warning("X-Plane found at IP: %s, port: %s" % (self.X_PLANE_IP,self.UDP_PORT))

    async def run(self) -> None:

        global rx_data

        logging.warning("Data streaming...\n----- Ctrl-C to exit -----")

        now = datetime.datetime.now()

        while not stop.is_set():
            try:
                self._time = time.time_ns()//1000 - self._boot_time
                rx_data[18][1] = time.time() - self._boot_time/1e6

                if self.rx_indices is not None:
                    for i in self.rx_indices:
                        rx_data[i][1] = 20*math.sin(self._time/2e6)

                now = datetime.datetime.now()
                rx_data[17][1] = (now - now.replace(hour=0, minute=0, second=0, microsecond=0)).total_seconds()
                rx_data[14][1] = 0

                try:
                    await asyncio.sleep(1 / self._freq)
                except asyncio.exceptions.CancelledError: 
                    stop.set()
                    raise
            except KeyboardInterrupt:
                stop.set()

            except Exception as e:
                logging.error("Socket error: ", e)

        self.close()

    def close(self) -> None:
        logging.debug('Closing XPL')
        logging.info('Stopped listening for drefs')
        logging.info('LUA suspended')

class ServoIO:
    def __init__(self, freq:int=FREQ) -> None:

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
            name="fmuas.xpinterface.servoio",
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

    async def run(self) -> None:

        def on_servo(msg: actuator.ArrayCommand_1, _:pycyphal.transport.TransferFrom) -> None:
            for index, command in enumerate(msg.commands[:TX_DATA_LAST_SERVO]):
                if command.command_type==actuator.Command_1.COMMAND_TYPE_POSITION:
                    tx_data[index][1] = math.degrees(command.command_value)
        self._sub_servo.receive_in_background(on_servo)

        while not stop.is_set():
            try:
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
            except asyncio.exceptions.CancelledError: 
                stop.set()
                raise
            except KeyboardInterrupt: stop.set()
            except Exception as e:
                logging.error(f'Error in SRV: {e}')

        await self.close()

    async def close(self) -> None:
        logging.debug('Closing SRV')
        self._node.close()

class ESCIO:
    def __init__(self, freq:int=FREQ) -> None:

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
            name="fmuas.xpinterface.escio",
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

    async def run(self) -> None:

        def on_esc(msg: esc.RawCommand_1, _:pycyphal.transport.TransferFrom) -> None:
            for index, value in enumerate(msg.cmd[:3]):
                tx_data[index + 1 + TX_DATA_LAST_SERVO][1] = value / 8192

        self._sub_esc.receive_in_background(on_esc)

        while not stop.is_set():
            try:
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
            except asyncio.exceptions.CancelledError: 
                stop.set()
                raise
            except KeyboardInterrupt: stop.set()
            except Exception as e:
                logging.error(f'Error in ESC: {e}')

        await self.close()

    async def close(self) -> None:
        logging.debug('Closing ESC')
        self._node.close()

class AttitudeSensor:

    def __init__(self, freq:int=FREQ) -> None:

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
            name="fmuas.xpinterface.attitudesensor",
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

    @staticmethod
    def euler_to_quaternion(roll:float, pitch:float, yaw:float) -> tuple[float, float, float, float]:
        """
        Convert an Euler angle to a quaternion. Copied 10/2/2023 from automaticaddison.com
        
        Input
            :param roll: The roll (rotation around x-axis) angle in radians.
            :param pitch: The pitch (rotation around y-axis) angle in radians.
            :param yaw: The yaw (rotation around z-axis) angle in radians.
        
        Output
            :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
        """
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        
        return [qx, qy, qz, qw]

    async def run(self) -> None:

        def on_time(msg: uavcan.time.SynchronizedTimestamp_1, _:pycyphal.transport.TransferFrom) -> None:
            self._time = msg.microsecond

        self._sub_clock_sync_time.receive_in_background(on_time)

        while not stop.is_set():
            try:
                
                m = ahrs.Solution_1(
                    uavcan_archived.Timestamp_1(self._time),
                    [q for q in AttitudeSensor.euler_to_quaternion(math.radians(rx_data[0][1]), math.radians(rx_data[1][1]), math.radians(rx_data[2][1]))],
                    [0.0],
                    [math.radians(rad) for rad in [rx_data[3][1],rx_data[4][1],rx_data[5][1]]],
                    [0.0],
                    [0.0,0.0,0.0],
                    [0.0]
                )

                try:
                    await self._pub_att.publish(m)
                    await asyncio.sleep(1 / self._freq)
                except asyncio.exceptions.CancelledError: 
                    stop.set()
                    raise

            except KeyboardInterrupt:
                stop.set()

            except Exception as e:
                logging.error(f'Error in ATT: {e}')

        await self.close()

    async def close(self) -> None:
        logging.debug('Closing ATT')
        self._node.close()

class AltitudeSensor:

    def __init__(self, freq:int=FREQ) -> None:

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
            name="fmuas.xpinterface.altitudesensor",
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

    async def run(self) -> None:

        def on_time(msg: uavcan.time.SynchronizedTimestamp_1, _:pycyphal.transport.TransferFrom) -> None:
            self._time = msg.microsecond

        self._sub_clock_sync_time.receive_in_background(on_time)

        while not stop.is_set():
            try:

                m = range_sensor.Measurement_1(
                    uavcan_archived.Timestamp_1(self._time),
                    0,
                    uavcan_archived.CoarseOrientation_1([0.0,0.0,0.0], True),
                    1.5,
                    range_sensor.Measurement_1.SENSOR_TYPE_RADAR,
                    range_sensor.Measurement_1.READING_TYPE_VALID_RANGE,
                    rx_data[6][1] * FT_TO_M
                )

                try:
                    await self._pub_alt.publish(m)
                    await asyncio.sleep(1 / self._freq)
                except asyncio.exceptions.CancelledError: 
                    stop.set()
                    raise

            except KeyboardInterrupt:
                stop.set()
                
            except Exception as e:
                logging.error(f'Error in ALT: {e}')

        await self.close()

    async def close(self) -> None:
        logging.debug('Closing ALT')
        self._node.close()

class GPSSensor:

    def __init__(self, freq:int=FREQ) -> None:

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
            name="fmuas.xpinterface.gpssensor",
        )

        self._node = pycyphal.application.make_node(node_info, self._registry)

        self._node.heartbeat_publisher.mode = uavcan.node.Mode_1.OPERATIONAL
        self._node.heartbeat_publisher.vendor_specific_status_code = os.getpid() % 100
        
        self._pub_gps = self._node.make_publisher(gnss.Fix2_1, 'gps')
        
        self._srv_sync_info = self._node.get_server(uavcan.time.GetSynchronizationMasterInfo_0, "gps_sync_info")
        self._srv_sync_info.serve_in_background(self._serve_sync_master_info)
        
        self._pub_gps_sync_time = self._node.make_publisher(uavcan.time.SynchronizedTimestamp_1, 'gps_sync_time')
        self._pub_gps_sync_time_last = self._node.make_publisher(uavcan.time.Synchronization_1, 'gps_sync_time')

        self._sub_clock_sync_time = self._node.make_subscriber(uavcan.time.SynchronizedTimestamp_1, 'clock_sync_time')
        # self._sub_clock_sync_time_last = self._node.make_subscriber(uavcan.time.Synchronization_1, 'clock_sync_time_last')

        # self._cln_clock_sync_info = self._node.make_client(uavcan.time.GetSynchronizationMasterInfo_0, config.getint('service_ids', 'clock_sync_info'), 'clock_sync_info')

        self._node.start()

    @staticmethod
    async def _serve_sync_master_info(request: uavcan.time.GetSynchronizationMasterInfo_0.Request, 
                                      metadata: pycyphal.presentation.ServiceRequestMetadata,) -> uavcan.time.GetSynchronizationMasterInfo_0.Response:
        logging.info("Execute command request %s from node %d", request, metadata.client_node_id)
        return uavcan.time.GetSynchronizationMasterInfo_0.Response(
            0.0, # error_variance
            uavcan.time.TimeSystem_0(uavcan.time.TimeSystem_0.TAI),
            uavcan.time.TAIInfo_0(uavcan.time.TAIInfo_0.DIFFERENCE_TAI_MINUS_GPS)
        )

    async def run(self) -> None:

        def on_time(msg: uavcan.time.SynchronizedTimestamp_1, _:pycyphal.transport.TransferFrom) -> None:
            self._time = msg.microsecond

        self._sub_clock_sync_time.receive_in_background(on_time)

        while not stop.is_set():
            try:
                
                gnss_time = int(1e6*(calendar.timegm(datetime.datetime.strptime(str(datetime.date.today().year), '%Y').timetuple()) + (1+rx_data[16][1])*86400 + rx_data[17][1]))

                m = gnss.Fix2_1(
                    uavcan_archived.Timestamp_1(self._time),
                    uavcan_archived.Timestamp_1(gnss_time),
                    gnss.Fix2_1.GNSS_TIME_STANDARD_GPS,
                    gnss.Fix2_1.NUM_LEAP_SECONDS_UNKNOWN, # TODO
                    int(rx_data[8][1]*1e8),
                    int(rx_data[7][1]*1e8),
                    0,
                    0,
                    [rx_data[9][1], rx_data[10][1], 0.0],
                    4,
                    gnss.Fix2_1.STATUS_2D_FIX,
                    gnss.Fix2_1.MODE_SINGLE,
                    0, # Submode
                    [0.0],
                    3.5,
                    gnss.ECEFPositionVelocity_1([0.0,0.0,0.0], [0,0,0], [0.0])
                )

                try:
                    await self._pub_gps.publish(m)
                    await asyncio.sleep(1 / self._freq)
                except asyncio.exceptions.CancelledError: 
                    stop.set()
                    raise

            except KeyboardInterrupt:
                stop.set()
                
            except Exception as e:
                logging.error(f'Error in GPS: {e}')

        await self.close()

    async def close(self) -> None:
        logging.debug('Closing GPS')
        self._node.close()

class IASSensor:

    def __init__(self, freq:int=FREQ) -> None:

        self._registry = pycyphal.application.make_registry(environment_variables={
            'UAVCAN__NODE__ID'              :config.get('node_ids', 'iassensor'),
            'UAVCAN__UDP__IFACE'            :config.get('main', 'udp'),

            'UAVCAN__PUB__IAS__ID'     :config.get('subject_ids', 'ias'),
        })

        self._freq = freq
        
        node_info = uavcan.node.GetInfo_1.Response(
            software_version=uavcan.node.Version_1(major=1, minor=0),
            name="fmuas.xpinterface.iassensor",
        )

        self._node = pycyphal.application.make_node(node_info, self._registry)

        self._node.heartbeat_publisher.mode = uavcan.node.Mode_1.OPERATIONAL
        self._node.heartbeat_publisher.vendor_specific_status_code = os.getpid() % 100
        
        self._pub_ias = self._node.make_publisher(air_data.IndicatedAirspeed_1, 'ias')

        self._node.start()

    async def run(self) -> None:

        while not stop.is_set():
            try:

                m = air_data.IndicatedAirspeed_1(
                    rx_data[11][1] * KT_TO_MS,
                    0.0
                )

                try:
                    await self._pub_ias.publish(m)
                    await asyncio.sleep(1 / self._freq)
                except asyncio.exceptions.CancelledError: 
                    stop.set()
                    raise

            except KeyboardInterrupt:
                stop.set()
                
            except Exception as e:
                logging.error(f'Error in IAS: {e}')

        await self.close()

    async def close(self) -> None:
        logging.debug('Closing IAS')
        self._node.close()

class AOASensor:

    def __init__(self, freq:int=FREQ) -> None:

        self._registry = pycyphal.application.make_registry(environment_variables={
            'UAVCAN__NODE__ID'              :config.get('node_ids', 'aoasensor'),
            'UAVCAN__UDP__IFACE'            :config.get('main', 'udp'),

            'UAVCAN__PUB__AOA__ID'     :config.get('subject_ids', 'aoa'),
        })

        self._freq = freq
        
        node_info = uavcan.node.GetInfo_1.Response(
            software_version=uavcan.node.Version_1(major=1, minor=0),
            name="fmuas.xpinterface.aoasensor",
        )

        self._node = pycyphal.application.make_node(node_info, self._registry)

        self._node.heartbeat_publisher.mode = uavcan.node.Mode_1.OPERATIONAL
        self._node.heartbeat_publisher.vendor_specific_status_code = os.getpid() % 100
        
        self._pub_aoa = self._node.make_publisher(air_data.AngleOfAttack_1, 'aoa')

        self._node.start()

    async def run(self) -> None:

        while not stop.is_set():
            try:

                m = air_data.AngleOfAttack_1(
                    air_data.AngleOfAttack_1.SENSOR_ID_LEFT,
                    math.radians(rx_data[12][1]),
                    0.0
                )

                try:
                    await self._pub_aoa.publish(m)
                    await asyncio.sleep(1 / self._freq)
                except asyncio.exceptions.CancelledError: 
                    stop.set()
                    raise

            except KeyboardInterrupt:
                stop.set()
                
            except Exception as e:
                logging.error(f'Error in AOA: {e}')

        await self.close()

    async def close(self) -> None:
        logging.debug('Closing AOA')
        self._node.close()

class SlipSensor:

    def __init__(self, freq:int=FREQ) -> None:

        self._registry = pycyphal.application.make_registry(environment_variables={
            'UAVCAN__NODE__ID'              :config.get('node_ids', 'slipsensor'),
            'UAVCAN__UDP__IFACE'            :config.get('main', 'udp'),

            'UAVCAN__PUB__SLIP__ID'     :config.get('subject_ids', 'slip'),
        })

        self._freq = freq
        
        node_info = uavcan.node.GetInfo_1.Response(
            software_version=uavcan.node.Version_1(major=1, minor=0),
            name="fmuas.xpinterface.slipsensor",
        )

        self._node = pycyphal.application.make_node(node_info, self._registry)

        self._node.heartbeat_publisher.mode = uavcan.node.Mode_1.OPERATIONAL
        self._node.heartbeat_publisher.vendor_specific_status_code = os.getpid() % 100
        
        self._pub_slip = self._node.make_publisher(air_data.Sideslip_1, 'slip')

        self._node.start()

    async def run(self) -> None:

        while not stop.is_set():
            try:

                m = air_data.Sideslip_1(
                    math.radians(rx_data[13][1]),
                    0.0
                )

                try:
                    await self._pub_slip.publish(m)
                    await asyncio.sleep(1 / self._freq)
                except asyncio.exceptions.CancelledError: 
                    stop.set()
                    raise

            except KeyboardInterrupt:
                stop.set()
                
            except Exception as e:
                logging.error(f'Error in SLP: {e}')

        await self.close()

    async def close(self) -> None:
        logging.debug('Closing SLP')
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
        
        node_info = uavcan.node.GetInfo_1.Response(
            software_version=uavcan.node.Version_1(major=1, minor=0),
            name="fmuas.xpinterface.clock",
        )

        self._node = pycyphal.application.make_node(node_info, self._registry)

        self._node.heartbeat_publisher.mode = uavcan.node.Mode_1.OPERATIONAL
        self._node.heartbeat_publisher.vendor_specific_status_code = os.getpid() % 100
        
        self._srv_sync_info = self._node.get_server(uavcan.time.GetSynchronizationMasterInfo_0, "clock_sync_info")
        self._srv_sync_info.serve_in_background(self._serve_sync_master_info)
        
        self._pub_sync_time = self._node.make_publisher(uavcan.time.SynchronizedTimestamp_1, 'clock_sync_time')
        self._pub_sync_time_last = self._node.make_publisher(uavcan.time.Synchronization_1, 'clock_sync_time_last')

        self._node.start()

    @staticmethod
    async def _serve_sync_master_info(request: uavcan.time.GetSynchronizationMasterInfo_0.Request, 
                                      metadata: pycyphal.presentation.ServiceRequestMetadata,)-> uavcan.time.GetSynchronizationMasterInfo_0.Response:
        logging.info("Execute command request %s from node %d", request, metadata.client_node_id)
        return uavcan.time.GetSynchronizationMasterInfo_0.Response(
            0.0, # error_variance
            uavcan.time.TimeSystem_0(uavcan.time.TimeSystem_0.MONOTONIC_SINCE_BOOT),
            uavcan.time.TAIInfo_0(uavcan.time.TAIInfo_0.DIFFERENCE_TAI_MINUS_UTC_UNKNOWN)
        )

    async def run(self) -> None:
        
        last_xpsecs = 0.0
        sync_secs = 0.0
        last_time = time.time()
        last_real = 0.0

        while not stop.is_set():
            try:
                try:
                    await self._pub_sync_time_last.publish(uavcan.time.Synchronization_1(int(self._sync_time))) # Last timestamp
                except asyncio.exceptions.CancelledError: 
                    stop.set()
                    raise

                xpsecs = rx_data[18][1]

                if abs(xpsecs - last_xpsecs) > 1.0: # time jump
                    logging.debug('XP time jumped')
                    sync_secs += 1e-6
                elif xpsecs == last_xpsecs and rx_data[14][1] != 1.0: # no time but running
                    sync_secs += time.time() - last_time
                elif rx_data[14][1] != 1.0: # normal and running
                    last_real += xpsecs - last_xpsecs
                    sync_secs = last_real

                last_time = time.time()
                self._sync_time = int(sync_secs*1e6)
                last_xpsecs = xpsecs

                try:
                    await self._pub_sync_time.publish(uavcan.time.SynchronizedTimestamp_1(int(self._sync_time))) # Current timestamp
                except asyncio.exceptions.CancelledError: 
                    stop.set()
                    raise

            except KeyboardInterrupt:
                stop.set()
                
            except Exception as e:
                logging.error(f'Error in CLK: {e}')

        await self.close()

    async def close(self) -> None:
        logging.debug('Closing CLK')
        self._node.close()

class Camera:

    def __init__(self, xpconnection:XPConnect, id:int=2, target_id:int=1) -> None:

        assert isinstance(xpconnection, XPConnect), 'Must pass an instance of XPConnect'
        self.sock = xpconnection.sock
        self.X_PLANE_IP = xpconnection.X_PLANE_IP
        self.UDP_PORT = xpconnection.UDP_PORT

        self.xp_path = config.get('xplane', 'xp_screenshot_path')
        
        self.camera_mav_conn = mavutil.mavlink_connection(config.get('mavlink', 'camera_conn'))
        
        assert isinstance(id,int) and 0<id<256, 'Camera ID must be UINT8'
        self.id = id
        
        assert isinstance(target_id,int) and 0<target_id<256, 'Camera target ID must be UINT8'
        self.target_id = target_id

    async def run(self) -> None:

        asyncio.create_task(self._heartbeat())

        while not stop.is_set():
            try:
                msg = self.camera_mav_conn.recv_msg()
                if msg is not None and msg.get_srcSystem()==self.target_id and msg.target_system==self.id and msg.get_type() == 'COMMAND_LONG':
                    if msg.command == mavutil.mavlink.MAV_CMD_IMAGE_START_CAPTURE:
                        cap = asyncio.create_task(self._capture_cycle(msg.param3, msg.param2))
                        self.camera_mav_conn.mav.command_ack_send(mavutil.mavlink.MAV_CMD_IMAGE_START_CAPTURE, mavutil.mavlink.MAV_RESULT_ACCEPTED, 255, 0, self.target_id, 0)
                    elif msg.command == mavutil.mavlink.MAV_CMD_IMAGE_STOP_CAPTURE:
                        try:
                            cap.cancel()
                            self.camera_mav_conn.mav.command_ack_send(mavutil.mavlink.MAV_CMD_IMAGE_STOP_CAPTURE, mavutil.mavlink.MAV_RESULT_ACCEPTED, 255, 0, self.target_id, 0)
                        except AttributeError:
                            pass
                            self.camera_mav_conn.mav.command_ack_send(mavutil.mavlink.MAV_CMD_IMAGE_STOP_CAPTURE, mavutil.mavlink.MAV_RESULT_DENIED, 255, 0, self.target_id, 0)
                    else:
                        self.camera_mav_conn.mav.command_ack_send(msg.command, mavutil.mavlink.MAV_RESULT_UNSUPPORTED, 255, 0, self.target_id, 0)


                await asyncio.sleep(0)

            except KeyboardInterrupt:
                self.close()
            except asyncio.exceptions.CancelledError:
                self.close()
                raise
                
            except Exception as e:
                logging.error(f'Error in CAM: {e}')

        self.close()

    async def _capture(self) -> None:
        
        delay = 0.5

        previousFileList=[f for f in os.listdir(self.xp_path) if os.path.isfile(os.path.join(self.xp_path, f))]

        self.sock.sendto(struct.pack("<4sx400s", b'CMND', b'sim/view/forward_with_nothing'), (self.X_PLANE_IP, self.UDP_PORT))
        self.sock.sendto(struct.pack("<4sx400s", b'CMND', b'sim/operation/screenshot'), (self.X_PLANE_IP, self.UDP_PORT))

        await asyncio.sleep(delay)
            
        newFileList = [f for f in os.listdir(self.xp_path) if os.path.isfile(os.path.join(self.xp_path, f))]

        fileDiff = [x for x in newFileList if x not in previousFileList]

        previousFileList = newFileList

        if len(fileDiff) != 0:
            os.chdir(self.xp_path)
            for f in fileDiff:
                shutil.move(f, './stored_images')
            print(fileDiff)

    async def _capture_cycle(self, iterations:int, period:float) -> None:
        
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

    async def _heartbeat(self) -> None:
        while not stop.is_set():
            try:
                self.camera_mav_conn.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_CAMERA,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    mavutil.mavlink.MAV_MODE_PREFLIGHT,
                    0,
                    mavutil.mavlink.MAV_STATE_ACTIVE
                )
                
                await asyncio.sleep(1)

            except KeyboardInterrupt:
                self.close()
            except asyncio.exceptions.CancelledError:
                self.close()
                raise

    def close(self) -> None:
        logging.info('Closing CAM')
        self.camera_mav_conn.close()

class TestCamera:

    def __init__(self, xpconnection:TestXPConnect, id:int=2, target_id:int=1) -> None:

        assert isinstance(xpconnection, TestXPConnect), 'Must pass an instance of TestXPConnect'

        self.xp_path = config.get('xplane', 'xp_screenshot_path')
        
        self.camera_mav_conn = mavutil.mavlink_connection(config.get('mavlink', 'camera_conn'))

        assert isinstance(id,int) and 0<id<256, 'Camera ID must be UINT8'
        self.id = id
        
        assert isinstance(target_id,int) and 0<target_id<256, 'Camera target ID must be UINT8'
        self.target_id = target_id

    async def run(self) -> None:

        asyncio.create_task(self._heartbeat())

        while not stop.is_set():
            try:
                msg = self.camera_mav_conn.recv_msg()
                if msg is not None and msg.get_srcSystem()==self.target_id and msg.target_system==self.id and msg.get_type() == 'COMMAND_LONG':
                    if msg.command == mavutil.mavlink.MAV_CMD_IMAGE_START_CAPTURE:
                        cap = asyncio.create_task(self._capture_cycle(msg.param3, msg.param2))
                        self.camera_mav_conn.mav.command_ack_send(mavutil.mavlink.MAV_CMD_IMAGE_START_CAPTURE, mavutil.mavlink.MAV_RESULT_ACCEPTED, 255, 0, self.target_id, 0)
                    elif msg.command == mavutil.mavlink.MAV_CMD_IMAGE_STOP_CAPTURE:
                        try:
                            cap.cancel()
                            self.camera_mav_conn.mav.command_ack_send(mavutil.mavlink.MAV_CMD_IMAGE_STOP_CAPTURE, mavutil.mavlink.MAV_RESULT_ACCEPTED, 255, 0, self.target_id, 0)
                        except AttributeError:
                            pass
                            self.camera_mav_conn.mav.command_ack_send(mavutil.mavlink.MAV_CMD_IMAGE_STOP_CAPTURE, mavutil.mavlink.MAV_RESULT_DENIED, 255, 0, self.target_id, 0)
                    else:
                        self.camera_mav_conn.mav.command_ack_send(msg.command, mavutil.mavlink.MAV_RESULT_UNSUPPORTED, 255, 0, self.target_id, 0)


                await asyncio.sleep(0)

            except KeyboardInterrupt:
                self.close()
            except asyncio.exceptions.CancelledError:
                self.close()
                raise
                
            except Exception as e:
                logging.error(f'Error in CAM: {e}')

    async def _capture(self) -> None:
        delay=0.5
        await asyncio.sleep(delay)

    async def _capture_cycle(self, iterations:int, period:float) -> None:
        
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

    async def _heartbeat(self) -> None:
        while not stop.is_set():
            try:
                self.camera_mav_conn.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_CAMERA,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    mavutil.mavlink.MAV_MODE_PREFLIGHT,
                    0,
                    mavutil.mavlink.MAV_STATE_ACTIVE
                )
                
                await asyncio.sleep(1)

            except KeyboardInterrupt:
                self.close()
            except asyncio.exceptions.CancelledError:
                self.close()
                raise
        
        self.close()

    def close(self) -> None:
        self.camera_mav_conn.close()

async def main():

    xpl = TestXPConnect() if os.name != 'nt' else XPConnect()
    cam = TestCamera(xpl) if os.name != 'nt' else Camera(xpl)
    clk = Clock()
    att = AttitudeSensor()
    alt = AltitudeSensor()
    gps = GPSSensor()
    ias = IASSensor()
    aoa = AOASensor()
    slp = SlipSensor()
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
        asyncio.create_task(slp.run()),
        asyncio.create_task(srv.run()),
        asyncio.create_task(esc.run()),

    ]

    try:
        await asyncio.gather(*tasks)
    except asyncio.exceptions.CancelledError: 
        stop.set()

    logging.warning('Nodes closed')

if __name__ == '__main__':
    asyncio.run(main())