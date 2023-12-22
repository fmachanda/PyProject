import os
import sys
import threading
import time
import tkinter as tk

import tkintermapview as tkmap
from tkinter import simpledialog

from gcs import Connect

stop = threading.Event()

HEARTBEAT_TIMEOUT = 5.0

os.chdir(os.path.dirname(os.path.realpath(__file__)) + '/..')
sys.path.append(os.getcwd())

from common.states import GlobalStates as g

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
                lat, lon, alt = MultiEntryDialog(self.root, ["Lat (deg)", "Lon (deg)", "Ft AGL"]).result
                alt = float(alt) if alt is not None else alt
            except (TypeError, ValueError):
                return

            if ConfirmationDialog(self.root, "Confirm landing?").result:
                self.connect_instance.v_land(lat=lat, lon=lon, alt=alt)
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
                alt = float(alt) if alt is not None else alt
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
            if alt is not None:
                self.connect_instance.set_alt(alt)
                self.log("Set alt")

    def show_set_speed_dialog(self):
        if self.connect_instance is not None:
            try:
                speed = float(MultiEntryDialog(self.root, ["KIAS"]).result[0])
            except TypeError:
                return
            if speed is not None:
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
