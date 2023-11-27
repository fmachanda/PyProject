import asyncio
import logging
import os
import sys
import time
from configparser import ConfigParser
from typing import Any

os.chdir(os.path.dirname(os.path.realpath(__file__)) + '/..')
sys.path.append(os.getcwd())
os.environ['MAVLINK20'] = '1'
os.environ['MAVLINK_DIALECT'] = 'common'

filehandler = logging.FileHandler('gcs/gcs.log', mode='a')
filehandler.setLevel(logging.INFO)
filehandler.setFormatter(logging.Formatter(str(os.getpid()) + ' (%(asctime)s %(name)s) %(levelname)s:%(message)s'))
streamhandler = logging.StreamHandler()
streamhandler.setLevel(logging.INFO)
logging.basicConfig(format='%(name)s %(levelname)s:%(message)s', level=logging.DEBUG, handlers=[filehandler, streamhandler])

from pymavlink import mavutil

import common.key as key

m = mavutil.mavlink

logging.getLogger('pymavlink').setLevel(logging.WARNING)
filehandler.setLevel(logging.DEBUG)

config = ConfigParser()
config.read('./common/CONFIG.ini')
mav_conn: mavutil.mavfile
systemid = 0
ids = []
mav_conn_open = False
MAX_FLUSH_BUFFER = int(1e6)
FT_TO_M = 0.3048
KT_TO_MS = 0.514444


class PreExistingConnection(Exception):
    """Exception subclass to prevent repeated MAVLINK Connection."""
    def __init__(self, message=f"A connection is already stored to this instance. Run Connect.close() method first."):
        """Inits the Excpetion class with a custom message."""
        self.message = message
        super().__init__(self.message)


def flush_buffer() -> None:
    """Removes any MAV messages still in the connection buffer"""
    # Buffer flusher limited by MAX_FLUSH_BUFFER to avoid loop.
    try:
        for i in range(MAX_FLUSH_BUFFER):
            msg = mav_conn.recv_match(blocking=False)
            if msg is None:
                logging.debug(f"Buffer flushed, {i} messages cleared")
                break
            elif i >= MAX_FLUSH_BUFFER-1:
                logging.error(f"Messages still in buffer after {MAX_FLUSH_BUFFER} flush cycles")     
    except (ConnectionError, OSError):
        logging.debug("No connection to flush.")


def check_mav_conn() -> None:
    """Checks if the global MAVLINK connection is open."""
    global mav_conn, mav_conn_open
    if not mav_conn_open:
        logging.info("Opening mav_conn")
        mav_conn = mavutil.mavlink_connection(config.get('mavlink', 'gcs_uav_conn'), source_system=systemid, input=True)
        mav_conn.setup_signing(key.KEY.encode('utf-8'))
        mav_conn_open = True


class Connect:
    """Runs a connection to a MAVLINK node on the UAV."""
    def __init__(self, target: int, timeout_cycles: int = 5) -> None:
        """Initialize the connection and send command change."""
        self.target = None
        assert 0<target<256 and isinstance(target, int) and target!=systemid, "System ID target must be unique UINT8"

        self.map_pos = [0.0, 0.0]
        
        self._heart_inhibit = False

        check_mav_conn()

        try:
            flush_buffer()
            
            logging.warning(f"Connecting to #{target}...")

            mav_conn.mav.change_operator_control_send(target, 0, 0, key.KEY.encode('utf-8'))
            time.sleep(1)

            for i in range(timeout_cycles):
                try:
                    msg = mav_conn.recv_msg()
                except ConnectionResetError:
                    raise PreExistingConnection

                if msg is not None and msg.get_type()=='CHANGE_OPERATOR_CONTROL_ACK' and msg.get_srcSystem()==target and msg.gcs_system_id==systemid:
                    match msg.ack:
                        case 0:
                            self.target = target        
                            if target not in ids:
                                ids.append(self.target)
                            logging.info(f"Connected to #{self.target}")
                            break
                        case 1 | 2:
                            logging.info(f"Bad connection key for #{target}")
                            break
                        case 3:
                            logging.info(f"#{target} is connected to another GCS")
                            break

                mav_conn.mav.change_operator_control_send(target, 0, 0, key.KEY.encode('utf-8'))
                time.sleep(1)

                if i >= timeout_cycles-1:
                    logging.warning("Connection failed (timeout)")

        except KeyboardInterrupt:
            self.close()
    
    async def _command(self, name: str, *params, acknowledge: bool = True, period: float = 1, timeout_cycles: int = 5) -> None:
        """Send a COMMAND_LONG message."""
        params_full = (list(params)+[0]*7)[:7]
        command = eval(f'm.MAV_CMD_{name}')

        try:
            flush_buffer()
            
            logging.warning(f"{name} sent to #{self.target}...")
            logging.debug(f"    {name} params: {params}")

            mav_conn.mav.command_long_send(
                self.target,
                0,
                command,
                0,
                float(params_full[0]),
                float(params_full[1]),
                float(params_full[2]),
                float(params_full[3]),
                float(params_full[4]),
                float(params_full[5]),
                float(params_full[6]),
            )

            if acknowledge:
                self._heart_inhibit = True
                await asyncio.sleep(period)

                for i in range(timeout_cycles):
                    msg = mav_conn.recv_msg()

                    if msg is not None and msg.get_type()=='COMMAND_ACK' and msg.get_srcSystem()==self.target and msg.target_system==systemid and msg.command==command:
                        match msg.result:
                            case m.MAV_RESULT_ACCEPTED | m.MAV_RESULT_IN_PROGRESS:
                                logging.info(f"{name} #{self.target}")
                                break
                            case m.MAV_RESULT_TEMPORARILY_REJECTED:
                                pass
                            # case m.MAV_RESULT_COMMAND_INT_ONLY:
                            #     self._command_int(name, *params)
                            #     break
                            case _:
                                logging.info(f"{name} failed on #{self.target}")
                                break

                    mav_conn.mav.command_long_send(
                        self.target,
                        0,
                        command,
                        0,
                        float(params_full[0]),
                        float(params_full[1]),
                        float(params_full[2]),
                        float(params_full[3]),
                        float(params_full[4]),
                        float(params_full[5]),
                        float(params_full[6]),
                    )

                    await asyncio.sleep(period)

                    if i >= timeout_cycles-1:
                        logging.warning(f"{name} on #{self.target} failed (timeout)")
                self._heart_inhibit = False
        except KeyboardInterrupt:
            self.close()

    async def _command_int(self, name: str, *params, acknowledge: bool = True, period: float = 1, timeout_cycles: int = 5, frame: int = m.MAV_FRAME_GLOBAL) -> None:
        """Send a COMMAND_INT message."""
        params_full = (list(params)+[0]*7)[:7]
        match name:
            case 'PID':
                command = 0
            case 'IMG':
                command = 1
            case _:
                command = eval(f'm.MAV_CMD_{name}')

        try:
            flush_buffer()
            
            logging.warning(f"{name} sent to #{self.target}...")
            logging.debug(f"    {name} params: {params}")

            mav_conn.mav.command_int_send(
                self.target,
                0,
                frame,
                command,
                0,0,
                float(params_full[0]),
                float(params_full[1]),
                float(params_full[2]),
                float(params_full[3]),
                int(float(params_full[4])),
                int(float(params_full[5])),
                int(float(params_full[6])),
            )

            if acknowledge:
                self._heart_inhibit = True
                await asyncio.sleep(period)
                for i in range(timeout_cycles):
                    msg = mav_conn.recv_msg()

                    if msg is not None and msg.get_type()=='COMMAND_ACK' and msg.get_srcSystem()==self.target and msg.target_system==systemid and msg.command==command:
                        match msg.result:
                            case m.MAV_RESULT_ACCEPTED | m.MAV_RESULT_IN_PROGRESS:
                                logging.info(f"{name} #{self.target}")
                                break
                            case m.MAV_RESULT_TEMPORARILY_REJECTED:
                                pass
                            case m.MAV_RESULT_COMMAND_INT_ONLY:
                                self._command(name, *params)
                                break
                            case _:
                                logging.info(f"{name} failed on #{self.target}")
                                break

                    mav_conn.mav.command_int_send(
                        self.target,
                        0,
                        frame,
                        command,
                        0,0,
                        float(params_full[0]),
                        float(params_full[1]),
                        float(params_full[2]),
                        float(params_full[3]),
                        int(float(params_full[4])),
                        int(float(params_full[5])),
                        int(float(params_full[6])),
                    )

                    await asyncio.sleep(period)

                    if i >= timeout_cycles-1:
                        logging.warning(f"{name} on #{self.target} failed (timeout)")
                self._heart_inhibit = False
        except KeyboardInterrupt:
            self.close()

    # region user functions
    def boot(self) -> None:
        """Set UAV mode to boot."""
        logging.info("Calling boot()")
        asyncio.run(self._command('DO_SET_MODE', m.MAV_MODE_PREFLIGHT, 1, 10))

    def set_mode(self, mode: int, custom_mode: int, custom_submode: int) -> None:
        """Set UAV mode."""
        logging.info("Calling set_mode()")
        asyncio.run(self._command('DO_SET_MODE', mode, custom_mode, custom_submode))

    def set_alt(self, alt: float) -> None:
        """Change UAV altitude setpoint."""
        logging.info("Calling set_alt()")
        asyncio.run(self._command('DO_CHANGE_ALTITUDE', alt, m.MAV_FRAME_GLOBAL_TERRAIN_ALT))

    def set_speed(self, airspeed: float) -> None:
        """Change UAV speed setpoint."""
        logging.info("Calling set_speed()")
        asyncio.run(self._command('DO_CHANGE_SPEED', m.SPEED_TYPE_AIRSPEED, airspeed*KT_TO_MS, -1))

    def reposition(self, lat: float | None = None, lon: float | None = None, alt: float = 0.0, speed: float = -1, radius: float = 0, yaw: float = 1):
        """Change UAV current waypoint."""
        logging.info("Calling reposition()")
        lat = self.map_pos[0] if lat is None else lat
        lon = self.map_pos[1] if lon is None else lon
        asyncio.run(self._command_int('DO_REPOSITION', speed, 0, radius, yaw, int(lat*1e7), int(lon*1e7), int(alt*FT_TO_M), acknowledge=False))

    def f_takeoff(self, latitude: float, longitude: float, altitude: float, yaw: float = float('nan'), pitch: float = 10):
        """Command UAV conventional takeoff."""
        logging.info("Calling f_takeoff()")
        asyncio.run(self._command_int('NAV_TAKEOFF', pitch, 0,0, yaw, int(latitude), int(longitude), int(altitude)))

    def v_takeoff(self, latitude: float, longitude: float, altitude: float, transit_heading: float = m.VTOL_TRANSITION_HEADING_TAKEOFF, yaw: float = float('nan')):
        """Command UAV vertical takeoff."""
        logging.info("Calling v_takeoff()")
        asyncio.run(self._command_int('NAV_VTOL_TAKEOFF', 0, transit_heading, 0, yaw, int(latitude), int(longitude), int(altitude)))
    
    def f_land(self, latitude: float, longitude: float, altitude: float, abort_alt: float = 0, yaw: float = float('nan')):
        """Command UAV conventional landing."""
        logging.info("Calling f_land()")
        asyncio.run(self._command_int('NAV_LAND', abort_alt, m.PRECISION_LAND_MODE_DISABLED, 0, yaw, int(latitude), int(longitude), int(altitude)))

    def v_land(self, latitude: float, longitude: float, altitude: float, approch_alt: float = float('nan'), yaw: float = float('nan')):
        """Command UAV vertical landing."""
        logging.info("Calling v_land()")
        asyncio.run(self._command_int('NAV_VTOL_LAND', m.NAV_VTOL_LAND_OPTIONS_DEFAULT, 0, approch_alt, yaw, int(latitude), int(longitude), int(altitude)))

    def gimbal_pitchyaw(self, pitch: float, yaw: float, pitchrate: float = 0.0, yawrate: float = 0.0, flags: int = m.GIMBAL_MANAGER_FLAGS_NEUTRAL, id: int = 0):
        """Command gimbal attitude."""
        logging.info("Calling gimbal_pitchyaw()")
        asyncio.run(self._command('DO_GIMBAL_MANAGER_PITCHYAW', pitch, yaw, pitchrate, yawrate, flags, id, acknowledge=False))

    def gimbal_roi_clear(self, id: int = 0):
        """Command gimbal to reset roi."""
        logging.info("Calling gimbal_roi_clear()")
        asyncio.run(self._command('DO_SET_ROI_NONE', id, acknowledge=False))

    def gimbal_roi(self, lat: float | None = None, lon: float | None = None, alt: float = 0.0, id: int = 0):
        """Command gimbal to roi."""
        logging.info("Calling gimbal_roi()")
        lat = self.map_pos[0] if lat is None else lat
        lon = self.map_pos[1] if lon is None else lon
        asyncio.run(self._command_int('DO_SET_ROI_LOCATION', id, 0, 0, 0, int(lat*1e7), int(lon*1e7), int(alt*FT_TO_M), acknowledge=False))

    def pid(self, kp: float, ti: float, td: float, setpoint: float):
        """DEVELOPMENT ONLY - send new PID parameters."""
        asyncio.run(self._command_int('PID',kp, ti, td, setpoint, 0.0, 0.0, 0.0, acknowledge=False))

    def img(self, interval: float = 1, num: float = 1, sequence: float = 0):
        """DEVELOPMENT ONLY - send screenshot command."""
        asyncio.run(self._command_int('IMG',interval, num, sequence, 0.0, int(0), int(0), int(0), acknowledge=False))
    # endregion

    def heartbeat(self) -> None:
        """Publish heartbeat message."""
        mav_conn.mav.heartbeat_send(
                    m.MAV_TYPE_GCS,
                    m.MAV_AUTOPILOT_INVALID,
                    m.MAV_MODE_PREFLIGHT,
                    0,
                    m.MAV_STATE_ACTIVE
                )

    def listen(self) -> bool:
        if self._heart_inhibit:
            return True
        
        try:
            msg = mav_conn.recv_msg()
        except (ConnectionError, OSError):
            logging.debug("No connection to listen to.")
            return
        
        if msg is not None:
            if msg.get_type() == 'HEARTBEAT':
                logging.debug(f"Heartbeat message from link #{msg.get_srcSystem()}")
            return msg.get_type()
        return False

    def close(self, because_heartbeat: bool = False) -> None:
        """Close the instance's MAVLINK connection."""
        global mav_conn_open

        flush_buffer()

        try:
            ids.remove(self.target)
            mav_conn.mav.change_operator_control_send(self.target, 1, 0, key.KEY.encode('utf-8'))
            if because_heartbeat:
                logging.warning("Heartbeat timeout")
            logging.warning("Closing GCS")

            if not ids:
                logging.info("Closing mav_conn")
                mav_conn.close()
                mav_conn_open = False
        except ValueError:
            logging.error("Connection not open")


if __name__=='__main__':
    logging.critical("Either run the GUI script or import this script to a python terminal.")
