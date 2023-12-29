import asyncio
import logging
import os
import sys
import threading
import time
import tkinter as tk
import tkintermapview as tkmap
from configparser import ConfigParser
from tkinter import simpledialog
from typing import Any

stop = threading.Event()

os.system('cls' if os.name == 'nt' else 'clear')
os.chdir(os.path.dirname(os.path.realpath(__file__)) + '/..')
sys.path.append(os.getcwd())
os.environ['MAVLINK20'] = '1'
os.environ['MAVLINK_DIALECT'] = 'common'

filehandler = logging.FileHandler('gcs/gcs.log', mode='a')
filehandler.setLevel(logging.INFO)
filehandler.setFormatter(logging.Formatter(str(os.getpid()) + " (%(asctime)s %(name)s) %(levelname)s:%(message)s"))
streamhandler = logging.StreamHandler()
streamhandler.setLevel(logging.INFO)
filehandler.setFormatter(logging.Formatter("(%(name)s)  %(levelname)s:%(message)s"))
logger = logging.getLogger("GCS")
logger.addHandler(filehandler)
logger.addHandler(streamhandler)
logger.setLevel(logging.DEBUG)

MAVLOG_DEBUG = logging.DEBUG - 3
MAVLOG_TX = logging.DEBUG - 2
MAVLOG_RX = logging.DEBUG - 1
MAVLOG_LOG = logging.INFO + 1

from pymavlink import mavutil

import common.key as key
from common.states import GlobalStates as g

m = mavutil.mavlink

logging.getLogger('pymavlink').setLevel(logging.WARNING)
filehandler.setLevel(logging.DEBUG)

config = ConfigParser()
config.read('./common/CONFIG.ini')
mav_conn: mavutil.mavfile
systemid = 0
ids = []
mav_conn_open = False

HEARTBEAT_TIMEOUT = 5.0
MAX_FLUSH_BUFFER = int(1e6)
FT_TO_M = 0.3048
KT_TO_MS = 0.514444

logging.addLevelName(MAVLOG_DEBUG, 'MAVdebug')
logging.addLevelName(MAVLOG_TX, 'TX')
logging.addLevelName(MAVLOG_RX, 'RX')
logging.addLevelName(MAVLOG_LOG, 'LOG')

_formatter = logging.Formatter(str(os.getpid()) + ' (%(asctime)s - %(name)s - %(levelname)s) %(message)s')
_filehandler = logging.FileHandler('mavlog.log', mode='a')
_filehandler.setFormatter(_formatter)

mavlogger = logging.getLogger(f'GCS{systemid}')
mavlogger.addHandler(_filehandler)
mavlogger.setLevel(MAVLOG_TX)
mavlogger.log(MAVLOG_TX, "test tx")
mavlogger.log(MAVLOG_RX, "test rx")
mavlogger.log(MAVLOG_LOG, "test log")


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
                logger.debug(f"Buffer flushed, {i} messages cleared")
                break
            elif i >= MAX_FLUSH_BUFFER-1:
                logger.error(f"Messages still in buffer after {MAX_FLUSH_BUFFER} flush cycles")     
    except (ConnectionError, OSError):
        logger.debug("No connection to flush.")


def check_mav_conn() -> None:
    """Checks if the global MAVLINK connection is open."""
    global mav_conn, mav_conn_open
    if not mav_conn_open:
        mavlogger.log(MAVLOG_TX, "Opening mav_conn")
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
        self.hl_data = None
        
        self._heart_inhibit = False

        check_mav_conn()

        try:
            flush_buffer()
            
            mavlogger.log(MAVLOG_TX, f"Connecting to #{target}...")

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
                            mavlogger.log(MAVLOG_RX, f"Connected to #{self.target}")
                            break
                        case 1 | 2:
                            mavlogger.log(MAVLOG_RX, f"Bad connection key for #{target}")
                            break
                        case 3:
                            mavlogger.log(MAVLOG_RX, f"#{target} is connected to another GCS")
                            break

                mav_conn.mav.change_operator_control_send(target, 0, 0, key.KEY.encode('utf-8'))
                time.sleep(1)

                if i >= timeout_cycles-1:
                    mavlogger.log(MAVLOG_TX, "Connection failed (timeout)")

        except KeyboardInterrupt:
            self.close()
    
    async def _command(self, name: str, *params, acknowledge: bool = False, period: float = 1, timeout_cycles: int = 5) -> None:
        """Send a COMMAND_LONG message."""
        params_full = (list(params)+[0]*7)[:7]
        command = eval(f'm.MAV_CMD_{name}')

        try:
            flush_buffer()
            
            mavlogger.log(MAVLOG_TX, f"{name} sent to #{self.target}...")
            mavlogger.log(MAVLOG_TX, f"    {name} params: {params}")

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
                                mavlogger.log(MAVLOG_RX, f"{name} #{self.target}")
                                break
                            case m.MAV_RESULT_TEMPORARILY_REJECTED:
                                pass
                            case _:
                                mavlogger.log(MAVLOG_RX, f"{name} failed on #{self.target}")
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
                        mavlogger.log(MAVLOG_TX, f"{name} on #{self.target} failed (timeout)")
                self._heart_inhibit = False
        except KeyboardInterrupt:
            self.close()

    async def _command_int(self, name: str, *params, acknowledge: bool = False, period: float = 1, timeout_cycles: int = 5, frame: int = m.MAV_FRAME_GLOBAL) -> None:
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
            
            mavlogger.log(MAVLOG_TX, f"{name} sent to #{self.target}...")
            mavlogger.log(MAVLOG_TX, f"    {name} params: {params}")

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
                                mavlogger.log(MAVLOG_RX, f"{name} #{self.target}")
                                break
                            case m.MAV_RESULT_TEMPORARILY_REJECTED:
                                pass
                            case _:
                                mavlogger.log(MAVLOG_RX, f"{name} failed on #{self.target}")
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
                        mavlogger.log(MAVLOG_TX, f"{name} on #{self.target} failed (timeout)")
                self._heart_inhibit = False
        except KeyboardInterrupt:
            self.close()

    # region user functions
    def boot(self) -> None:
        """Set UAV mode to boot."""
        logger.info("Calling boot()")
        asyncio.run(self._command('DO_SET_MODE', m.MAV_MODE_PREFLIGHT, 1, 10))

    def set_mode(self, custom_submode: int | str, mode: int | None = None, custom_mode: int | None = None) -> None:
        """Set UAV mode."""
        logger.info("Calling set_mode()")
        try:
            custom_submode = int(custom_submode)
        except ValueError:
            try:
                custom_submode = getattr(g, f"CUSTOM_SUBMODE_{custom_submode.upper()}")
            except AttributeError:
                logger.warning("Unrecognized submode requested")
                return
        mode = g.ALLOWED_MODES.get(custom_submode)[0] if mode is None else mode
        custom_mode = g.ALLOWED_CUSTOM_MODES.get(custom_submode)[0] if custom_mode is None else custom_mode
        asyncio.run(self._command('DO_SET_MODE', mode, custom_mode, custom_submode))

    def set_alt(self, alt: float) -> None:
        """Change UAV alt setpoint."""
        logger.info("Calling set_alt()")
        asyncio.run(self._command('DO_CHANGE_ALTITUDE', alt, m.MAV_FRAME_GLOBAL_TERRAIN_ALT))

    def set_speed(self, airspeed: float) -> None:
        """Change UAV speed setpoint."""
        logger.info("Calling set_speed()")
        asyncio.run(self._command('DO_CHANGE_SPEED', 0, airspeed*KT_TO_MS, -1))

    def reposition(self, lat: float | None = None, lon: float | None = None, alt: float | None = None, speed: float = -1, radius: float = 0, yaw: float = 1):
        """Change UAV current waypoint."""
        logger.info("Calling reposition()")
        lat = self.map_pos[0] if lat is None else lat
        lon = self.map_pos[1] if lon is None else lon
        alt = 50.0 if alt is None else alt
        asyncio.run(self._command_int('DO_REPOSITION', speed, 0, radius, yaw, int(lat*1e7), int(lon*1e7), int(alt*FT_TO_M), acknowledge=False))

    def v_takeoff(self, lat: float, lon: float, alt: float, transit_heading: float = m.VTOL_TRANSITION_HEADING_NEXT_WAYPOINT, yaw: float = float('nan')):
        """Command UAV vertical takeoff."""
        logger.info("Calling v_takeoff()")
        asyncio.run(self._command_int('NAV_VTOL_TAKEOFF', 0, transit_heading, 0, yaw, int(lat), int(lon), int(alt)))
    
    def f_land(self, lat: float, lon: float, alt: float, abort_alt: float = 0, yaw: float = float('nan')):
        """Command UAV conventional landing."""
        lat = self.map_pos[0] if lat is None else lat
        lon = self.map_pos[1] if lon is None else lon
        alt = 0.0 if alt is None else alt
        asyncio.run(self._command_int('NAV_LAND', abort_alt, m.PRECISION_LAND_MODE_DISABLED, 0, yaw, int(lat), int(lon), int(alt)))

    def v_land(self, lat: float, lon: float, alt: float, approch_alt: float = float('nan'), yaw: float = float('nan')):
        """Command UAV vertical landing."""
        logger.info("Calling v_land()")
        lat = self.map_pos[0] if lat is None else lat
        lon = self.map_pos[1] if lon is None else lon
        alt = 0.0 if alt is None else alt
        asyncio.run(self._command_int('NAV_VTOL_LAND', m.NAV_VTOL_LAND_OPTIONS_DEFAULT, 0, approch_alt, yaw, int(lat), int(lon), int(alt)))

    def gimbal_pitchyaw(self, pitch: float, yaw: float, pitchrate: float = 0.0, yawrate: float = 0.0, flags: int = m.GIMBAL_MANAGER_FLAGS_NEUTRAL, id: int = 0):
        """Command gimbal attitude."""
        logger.info("Calling gimbal_pitchyaw()")
        asyncio.run(self._command('DO_GIMBAL_MANAGER_PITCHYAW', pitch, yaw, pitchrate, yawrate, flags, id, acknowledge=False))

    def gimbal_roi_clear(self, id: int = 0):
        """Command gimbal to reset roi."""
        logger.info("Calling gimbal_roi_clear()")
        asyncio.run(self._command('DO_SET_ROI_NONE', id, acknowledge=False))

    def gimbal_roi(self, lat: float | None = None, lon: float | None = None, alt: float | None = None, id: int = 0):
        """Command gimbal to roi."""
        logger.info("Calling gimbal_roi()")
        lat = self.map_pos[0] if lat is None else lat
        lon = self.map_pos[1] if lon is None else lon
        alt = 0.0 if alt is None else alt
        asyncio.run(self._command_int('DO_SET_ROI_LOCATION', id, 0, 0, 0, int(lat*1e7), int(lon*1e7), int(alt*FT_TO_M), acknowledge=False))

    def pid(self, id: int, kp: float, ti: float, td: float, setpoint: float):
        """DEVELOPMENT ONLY - send new PID parameters."""
        asyncio.run(self._command_int('PID', kp, ti, td, setpoint, id, 0, 0, acknowledge=False))

    def img(self, id: float = 0, interval: float = 1, num: float = 1, sequence: float = 0):
        """Send image command."""
        asyncio.run(self._command_int('IMAGE_START_CAPTURE', id, interval, num, sequence, int(0), int(0), int(0), acknowledge=False))
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
            mavlogger.log(MAVLOG_DEBUG, "No connection to listen to.")
            return
        
        if msg is not None:
            if msg.get_type() == 'HEARTBEAT':
                mavlogger.log(MAVLOG_DEBUG, f"Heartbeat message from link #{msg.get_srcSystem()}")
            if msg.get_type() == 'HIGH_LATENCY2':
                mavlogger.log(MAVLOG_DEBUG, f"HL2 message from link #{msg.get_srcSystem()}")
                self.hl_data = msg
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
                mavlogger.log(MAVLOG_LOG, "Heartbeat timeout")
            mavlogger.log(MAVLOG_LOG, "Closing GCS")

            if not ids:
                mavlogger.log(MAVLOG_LOG, "Closing mav_conn")
                mav_conn.close()
                mav_conn_open = False
        except ValueError:
            mavlogger.log(MAVLOG_RX, "Connection not open")


class GCSUI:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("GCS Control")
        self.root.protocol("WM_DELETE_WINDOW", self.window_close)

        self.map_open = False
        self.map_marker = None
        self.pos_marker = None

        self.status_label = tk.Label(root, text="Status: Booting...", bd=1, relief=tk.SUNKEN, anchor=tk.W)
        self.status_label.pack(side=tk.BOTTOM, fill=tk.X)

        self.telem_label = tk.Label(root, text="MODE: nan    BATT: nan    KIAS: nan    FT AGL: nan    VS: nan    ", bd=1, relief=tk.SUNKEN, anchor=tk.W)
        self.telem_label.pack(side=tk.BOTTOM, fill=tk.X)

        self.log_text = tk.Text(root, height=10, width=40, state=tk.DISABLED)
        self.log_text.pack(padx=10, pady=(10, 8))

        self.map_button = tk.Button(root, text="Open Map", command=self.open_map)
        self.map_button.pack(pady=(0, 10))

        self.root.bind('<Return>', self.return_key)

        # Create a menu bar
        self.menubar = tk.Menu(root)
        self.root.config(menu=self.menubar)

        self.autopilot_menu = tk.Menu(self.menubar, tearoff=0)
        self.connect_menu = tk.Menu(self.menubar, tearoff=0)
        self.mode_menu = tk.Menu(self.menubar, tearoff=0)
        self.sp_menu = tk.Menu(self.autopilot_menu, tearoff=0)
        self.nav_menu = tk.Menu(self.autopilot_menu, tearoff=0)
        self.cam_menu = tk.Menu(self.menubar, tearoff=0)
        self.dev_menu = tk.Menu(self.menubar, tearoff=0)

        self.menubar.add_cascade(label="Connection", menu=self.connect_menu)
        self.menubar.add_cascade(label="Mode", menu=self.mode_menu, state=tk.DISABLED)
        self.menubar.add_cascade(label="Autopilot", menu=self.autopilot_menu, state=tk.DISABLED)
        self.menubar.add_cascade(label="Camera", menu=self.cam_menu, state=tk.DISABLED)
        self.menubar.add_cascade(label="DEV", menu=self.dev_menu, state=tk.DISABLED)

        self.connect_menu.add_command(label="Connect", command=self.connect)
        self.connect_menu.add_command(label="Close", command=self.close)

        self.mode_menu.add_command(label="Set mode", command=self.show_set_mode_dialog)
        self.mode_menu.add_command(label="Boot", command=self.mode_boot)
        self.mode_menu.add_command(label="Arm", command=self.mode_arm)
        self.mode_menu.add_command(label="Disarm", command=self.mode_disarm)
        self.mode_menu.add_separator()
        self.mode_menu.add_command(label="Emergency Abort", command=self.show_emergency_abort_dialog, state=tk.DISABLED)

        self.autopilot_menu.add_command(label="Set altitude", command=self.show_set_alt_dialog)
        self.autopilot_menu.add_command(label="Set speed", command=self.show_set_speed_dialog)
        self.autopilot_menu.add_command(label="Direct to", command=self.show_direct_to_dialog)
        self.autopilot_menu.add_command(label="Takeoff", command=self.show_takeoff_dialog)
        self.autopilot_menu.add_command(label="Land", command=self.show_landing_dialog)

        self.cam_menu.add_command(label="Take single image", command=self.cam_take_single_image)
        self.cam_menu.add_command(label="Take images", command=self.show_take_image_dialog)
        self.cam_menu.add_command(label="Pitch/yaw", command=self.show_pitchyaw_dialog)
        self.cam_menu.add_command(label="ROI", command=self.show_roi_dialog)
        self.cam_menu.add_command(label="ROI Clear", command=self.cam_roi_clear)
        self.cam_menu.add_command(label="Stow", command=self.cam_stow)

        self.dev_menu.add_command(label="PID", command=self.show_pid_tuner_dialog)

        self.connect_instance: Connect | None = None
        self.heartbeat = None
        from utilities import pid_tune_map as p; self.pid_defaults = p

        self.update_status()

        heart = threading.Thread(target=self.heartbeat, daemon=True)
        heart.start()
        listener = threading.Thread(target=self.listen, daemon=True)
        listener.start()

    def enable_menus(self):
        if self.connect_instance is None:
            co = tk.DISABLED
            contra = tk.NORMAL
        else:
            co = tk.NORMAL
            contra = tk.DISABLED

        self.connect_menu.entryconfig("Connect", state=contra)
        self.menubar.entryconfig("Mode", state=co)
        self.menubar.entryconfig("Autopilot", state=co)
        self.menubar.entryconfig("Camera", state=co)
        self.menubar.entryconfig("DEV", state=co)

    def open_map(self):
        if not self.map_open:
            self.map = tk.Toplevel(self.root)
            self.map.title("Map")
            self.map_widget = tkmap.TkinterMapView(self.map, width=800, height=600, corner_radius=0)
            self.map_widget.place(relx=0.5, rely=0.5, anchor=tk.CENTER)
            self.map_widget.set_position(18.5622536, -72.2842880)
            self.map_widget.set_zoom(9)
            self.map_widget.add_right_click_menu_command(label="Add Marker", command=self.add_map_marker, pass_coords=True)
            self.map_widget.add_right_click_menu_command(label="Remove Marker", command=self.delete_map_marker, pass_coords=True)
            self.map.protocol("WM_DELETE_WINDOW", self.close_map)
            self.map_open = True
        else:
            self.map.lift()

    def close_map(self):
        self.map.destroy()
        self.map_open = False

    def add_map_marker(self, coords: tuple):
        self.delete_map_marker()
        self.map_marker = self.map_widget.set_marker(coords[0], coords[1])

    def delete_map_marker(self, _: tuple | None = None):
        if self.map_marker:
            try:
                self.map_marker.delete()
            except tk.TclError:
                pass
            finally:
                self.map_marker = None

    #region Dialogs
    def show_takeoff_dialog(self):
        if self.connect_instance is not None and ConfirmationDialog(self.root, "Confirm takeoff?").result:
            # TODO: lat/lon/alt alignment
            self.connect_instance.v_takeoff(0.0, 0.0, 0.0)
            self.log("Takeoff")

    def show_landing_dialog(self):
        if self.connect_instance is not None:
            try:
                self.connect_instance.map_pos = list(self.map_marker.position)
            except AttributeError:
                self.connect_instance.map_pos = [0.0, 0.0]

            try:
                lat, lon = MultiEntryDialog(self.root, ["Lat (deg)", "Lon (deg)"]).result
            except (TypeError, ValueError):
                return

            if ConfirmationDialog(self.root, "Confirm landing?").result:
                self.connect_instance.v_land(lat=lat, lon=lon, alt=0.0)
                self.log("Landing")

    def show_emergency_abort_dialog(self):
        if self.connect_instance is not None:
            try:
                self.connect_instance.map_pos = list(self.map_marker.position)
            except AttributeError:
                self.connect_instance.map_pos = [0.0, 0.0]

            try:
                lat, lon, alt = MultiEntryDialog(self.root, ["Lat (deg)", "Lon (deg)", "Ft AGL"]).result
                alt = float(alt) if alt is not None else alt
            except (TypeError, ValueError):
                return

            if not ConfirmationDialog(self.root, "Confirm emergency abort?").result:
                return
            
            if ConfirmationDialog(self.root, "WARNING!\n\nThis action will command a crash landing.\n\nEnsure persons and property on\n ground will not be endangered.").result:
                self.connect_instance.f_land(lat=lat, lon=lon, alt=alt)
                self.log("EMERGENCY LANDING")

    def show_set_mode_dialog(self):
        if self.connect_instance is not None:
            try:
                # Do not convert to int
                mode = MultiEntryDialog(self.root, ["Mode"]).result[0]
            except TypeError:
                return
            if mode is not None:
                self.connect_instance.set_mode(mode)
                self.log("Set mode")

    def show_direct_to_dialog(self):
        if self.connect_instance is not None:
            try:
                self.connect_instance.map_pos = list(self.map_marker.position)
            except AttributeError:
                self.connect_instance.map_pos = [0.0, 0.0]

            try:
                lat, lon, alt = MultiEntryDialog(self.root, ["Lat (deg)", "Lon (deg)", "Ft AGL"]).result
                lat = float(lat)
                lon = float(lon)
                alt = -1 if alt is None else float(alt)
            except (TypeError, ValueError):
                return
            self.connect_instance.reposition(lat=lat, lon=lon, alt=alt)
            self.log("Direct to")

    def show_set_alt_dialog(self):
        if self.connect_instance is not None:
            try:
                alt = float(MultiEntryDialog(self.root, ["Ft AGL"]).result[0])
            except TypeError:
                return
            
            alt = -2 if alt is None else alt
            self.connect_instance.set_alt(alt)
            self.log("Set alt")
            

    def show_set_speed_dialog(self):
        if self.connect_instance is not None:
            try:
                speed = float(MultiEntryDialog(self.root, ["KIAS"]).result[0])
            except TypeError:
                return
            speed = -2 if speed is None else speed
            self.connect_instance.set_speed(speed)
            self.log("Set speed")

    def show_pitchyaw_dialog(self):
        if self.connect_instance is not None:
            try:
                p, y = MultiEntryDialog(self.root, ["Pitch", "Yaw"]).result
                p = float(p)
                y = float(y)
            except (TypeError, ValueError):
                return
            self.connect_instance.gimbal_pitchyaw(p, y)
            self.log("Pitchyaw")

    def show_roi_dialog(self):
        if self.connect_instance is not None:
            try:
                self.connect_instance.map_pos = list(self.map_marker.position)
            except AttributeError:
                self.connect_instance.map_pos = [0.0, 0.0]

            try:
                lat, lon, alt = MultiEntryDialog(self.root, ["Lat (deg)", "Lon (deg)", "Ft AGL"]).result
                alt = float(alt) if alt is not None else alt
            except (TypeError, ValueError):
                return
            self.connect_instance.gimbal_roi(lat=lat, lon=lon, alt=alt)
            self.log("ROI")

    def show_take_image_dialog(self):
        if self.connect_instance is not None:
            try:
                id, interval, num, sequence = MultiEntryDialog(self.root, ["ID", "Interval", "Number", "Sequence"]).result
                id = 0 if id is None else id
                interval = 1 if interval is None else interval
                num = 1 if num is None else num
                sequence = 0 if sequence is None else sequence
            except (TypeError, ValueError):
                return
            self.connect_instance.img(id=id, sequence=sequence, num=num, interval=interval)
            self.log("Take images")

    def show_pid_tuner_dialog(self):
        self.init_pid_window()
    #endregion

    def cam_take_single_image(self):
        if self.connect_instance is not None:
            self.connect_instance.img()
            self.log("Take single image")

    def cam_roi_clear(self):
        if self.connect_instance is not None:
            self.connect_instance.gimbal_roi_clear()
            self.log("ROI Clear")

    def cam_stow(self):
        if self.connect_instance is not None:
            self.connect_instance.gimbal_pitchyaw(-180, 0)
            self.log("Stow")

    def return_key(self, event=None):
        if self.connect_instance is None:
            self.connect()

    def connect(self):
        try:
            target_id = int(MultiEntryDialog(self.root, ["Target ID"]).result[0])
        except (TypeError, ValueError):
            self.log("Error: ID must be UINT8")
            return

        if isinstance(target_id, int) and target_id<256:
            if self.connect_instance is None:
                self.connect_instance = Connect(target_id)
                if self.connect_instance.target is not None:
                    self.log(f"Connected to target #{target_id}")
                    self.enable_menus()
                else:
                    self.connect_instance = None
                    self.log("Connection timeout")
            elif self.connect_instance.target != target_id:
                self.log(f"Close instance #{self.connect_instance.target} first")
            else:
                pass
        else:
            self.log("Error: ID must be UINT8")

        self.update_status()

    def mode_boot(self):
        if self.connect_instance is not None:
            self.connect_instance.boot()
            self.log("Boot")

    def mode_arm(self):
        if self.connect_instance is not None:
            self.connect_instance.set_mode("GROUND_ARMED")
            self.log("Arm")

    def mode_disarm(self):
        if self.connect_instance is not None:
            self.connect_instance.set_mode("GROUND_DISARMED")
            self.log("Disarm")

    def update_status(self):
        message = "Unconnected" if self.connect_instance is None else f"Connected to UAV{self.connect_instance.target}"
        self.status_label.config(text=f"Status: {message}", bg=('red' if self.connect_instance is None else 'green'))

    def close(self, because_heartbeat: bool = False):
        if self.connect_instance is not None:
            self.connect_instance.close(because_heartbeat)
            self.connect_instance = None
            self.log("Closing...")
        else:
            self.log("Connect first")
        
        self.update_status()
        self.enable_menus()

    def window_close(self):
        self.close()
        stop.set()
        self.root.destroy()

    def log(self, message):
        self.log_text.config(state=tk.NORMAL)
        self.log_text.insert(tk.END, message + '\n')
        self.log_text.see(tk.END)
        self.log_text.config(state=tk.DISABLED)

    def heartbeat(self) -> None:
        """Publish heartbeat message periodically in background."""
        try:
            while not stop.is_set():
                if self.connect_instance is not None:
                    self.connect_instance.heartbeat()
                time.sleep(1)
        except KeyboardInterrupt:
            return

    def listen(self) -> None:
        try:
            last_heartbeat = 0.0
            while not stop.is_set():
                if self.connect_instance is not None:
                    result=self.connect_instance.listen()
                    if result == 'HEARTBEAT':
                        last_heartbeat = time.time()
                    if time.time() - last_heartbeat > HEARTBEAT_TIMEOUT:
                        self.log(f"Heartbeat timeout from #{self.connect_instance.target}")
                        self.close(because_heartbeat=True)

                    if result == 'HIGH_LATENCY2':
                        self.telem_label.config(text=f"MODE: {g.CUSTOM_SUBMODE_NAMES.get(self.connect_instance.hl_data.custom0, "Unknown")}    BATT: {self.connect_instance.hl_data.battery}%    KIAS: {int(self.connect_instance.hl_data.airspeed*0.388768)}    FT AGL: {int((self.connect_instance.hl_data.custom1+128)*32.8084)}    VS: {int(19.685*self.connect_instance.hl_data.climb_rate)}    ")
                        try:
                            if not hasattr(self, 'map_widget'):
                                continue
                            if self.pos_marker:
                                self.pos_marker.delete()
                            self.pos_marker = None
                            self.pos_marker = self.map_widget.set_marker(
                                self.connect_instance.hl_data.latitude * 1e-7,
                                self.connect_instance.hl_data.longitude * 1e-7,
                                text = f"UAV{self.connect_instance.target}",
                                text_color = "#151f29",
                                marker_color_circle = "#293847",
                                marker_color_outside = "#485a6e",
                            )
                        except tk.TclError:
                            pass
                time.sleep(0)
        except KeyboardInterrupt:
            return

    #region PID Tuner
    def apply_tuning(self, event=None):
        if self.connect_instance is not None:
            selected_pid = self.pid_var.get()
            kp = float(self.kp_entry.get()) if self.kp_entry.get() else None
            ti = float(self.ti_entry.get()) if self.ti_entry.get() else None
            td = float(self.td_entry.get()) if self.td_entry.get() else None
            sp = float(self.sp_entry.get()) if self.sp_entry.get() else None

            self.pid_defaults[selected_pid].kp = kp
            self.pid_defaults[selected_pid].ti = ti
            self.pid_defaults[selected_pid].td = td
            self.pid_defaults[selected_pid].sp = sp
            id = self.pid_defaults[selected_pid].id

            kp = 0.0 if kp is None else kp
            ti = 0.0 if ti is None else ti
            td = 0.0 if td is None else td
            sp = 0.0 if sp is None else sp

            self.connect_instance.pid(id, kp, ti, td, sp)
            self.log(f"{selected_pid}, {kp}, {ti}, {td} @ {sp}")

    def init_pid_window(self):
        self.pid_tuner_window = tk.Toplevel(self.root)
        self.pid_tuner_window.title("PID Tuner")

        self.pid_var = tk.StringVar(self.pid_tuner_window)
        self.pid_var.set(next(iter(self.pid_defaults.keys())))

        self.pid_warning = tk.Label(self.pid_tuner_window, text="hic sunt dracones!")
        self.pid_warning.pack()

        self.pid_dropdown = tk.OptionMenu(self.pid_tuner_window, self.pid_var, *self.pid_defaults.keys())
        self.pid_dropdown.pack(pady=10)

        self.kp_entry = tk.Entry(self.pid_tuner_window)
        self.ti_entry = tk.Entry(self.pid_tuner_window)
        self.td_entry = tk.Entry(self.pid_tuner_window)
        self.sp_entry = tk.Entry(self.pid_tuner_window)

        tk.Label(self.pid_tuner_window, text="KP:").pack()
        self.kp_entry.pack()

        tk.Label(self.pid_tuner_window, text="TI:").pack()
        self.ti_entry.pack()

        tk.Label(self.pid_tuner_window, text="TD:").pack()
        self.td_entry.pack()

        tk.Label(self.pid_tuner_window, text="Setpoint (SP):").pack()
        self.sp_entry.pack()

        self.apply_button = tk.Button(self.pid_tuner_window, text="Apply Tuning", command=self.apply_tuning)
        self.apply_button.pack(pady=10)

        self.pid_var.trace("w", self.update_entries)

        self.pid_tuner_window.bind("<Return>", self.apply_tuning)
        self.pid_tuner_window.bind("<Escape>", self.close_pid_tuner)

        self.update_entries()

    def update_entries(self, *args):
        selected_pid = self.pid_var.get()
        default_pid = self.pid_defaults[selected_pid]

        self.kp_entry.delete(0, tk.END)
        self.ti_entry.delete(0, tk.END)
        self.td_entry.delete(0, tk.END)
        self.sp_entry.delete(0, tk.END)

        self.kp_entry.insert(0, "" if default_pid.kp is None else str(default_pid.kp))
        self.ti_entry.insert(0, "" if default_pid.ti is None else str(default_pid.ti))
        self.td_entry.insert(0, "" if default_pid.td is None else str(default_pid.td))
        self.sp_entry.insert(0, "" if default_pid.sp is None else str(default_pid.sp))

    def close_pid_tuner(self, event=None):
        self.pid_tuner_window.destroy()
    #endregion


class MultiEntryDialog(simpledialog.Dialog):
    def __init__(self, parent, entry_names=None, **kwargs):
        self.entry_names = entry_names or []
        self.num_entries = len(self.entry_names)
        super().__init__(parent, **kwargs)

    def body(self, master):
        tk.Label(master, text="Enter values:").grid(row=0, column=0, columnspan=2, pady=10)

        self.entries = []
        for i, entry_name in enumerate(self.entry_names):
            tk.Label(master, text=f"{entry_name}:").grid(row=i + 1, column=0, sticky='e')
            entry = tk.Entry(master)
            entry.grid(row=i + 1, column=1, padx=10)
            self.entries.append(entry)

        return self.entries[0]  # Return the first entry widget for initial focus

    def buttonbox(self):
        box = tk.Frame(self)

        tk.Button(box, text="OK", command=self.ok).pack(side='left', padx=5, pady=5)
        tk.Button(box, text="Cancel", command=self.cancel).pack(side='left', padx=5, pady=5)

        self.bind("<Return>", self.ok)
        self.bind("<Escape>", self.cancel)

        box.pack()

    def apply(self):
        # This method is called when the "OK" button is clicked
        self.result = [entry.get() if entry.get() else None for entry in self.entries]


class ConfirmationDialog(simpledialog.Dialog):
    def __init__(self, parent, text, **kwargs):
        self.text = text
        self.ok_pressed = False
        super().__init__(parent, **kwargs)

    def body(self, master):
        tk.Label(master, text=self.text).pack(padx=10, pady=10)
        return tk.Frame(master)  # No input elements, so return an empty frame

    def buttonbox(self):
        box = tk.Frame(self)

        tk.Button(box, text="OK", command=self.ok).pack(side='left', padx=5, pady=5)
        tk.Button(box, text="Cancel", command=self.cancel).pack(side='left', padx=5, pady=5)

        self.bind("<Return>", self.ok)
        self.bind("<Escape>", self.cancel)

        box.pack()

    def ok(self, event=None):
        self.ok_pressed = True
        super().ok()

    def cancel(self, event=None):
        self.result = self.ok_pressed
        super().cancel()


def main() -> None:
    app = GCSUI(tk.Tk())

    try:
        app.root.mainloop()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()

