# import asyncio
import re
import threading
import time
import tkinter as tk

import tkintermapview as tkmap
from tkinter import simpledialog

from gcs import Connect

stop = threading.Event()

HEARTBEAT_TIMEOUT = 2.0


class GCSUI:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("GCS Control")
        self.root.protocol("WM_DELETE_WINDOW", self.window_close)

        self.map_open = False
        self.map_marker = None

        self.status_label = tk.Label(root, text="Status: Booting...", bd=1, relief=tk.SUNKEN, anchor=tk.W)
        self.status_label.pack(side=tk.BOTTOM, fill=tk.X)

        self.log_text = tk.Text(root, height=10, width=40)
        self.log_text.pack(padx=10, pady=10)

        self.map_button = tk.Button(root, text="Open Map", command=self.open_map)
        self.map_button.pack(pady=10)

        # Create a menu bar
        self.menubar = tk.Menu(root)
        self.root.config(menu=self.menubar)

        self.autopilot_menu = tk.Menu(self.menubar, tearoff=0)
        self.connect_menu = tk.Menu(self.menubar, tearoff=0)
        self.mode_menu = tk.Menu(self.menubar, tearoff=0)
        self.sp_menu = tk.Menu(self.autopilot_menu, tearoff=0)
        self.nav_menu = tk.Menu(self.autopilot_menu, tearoff=0)
        self.cam_menu = tk.Menu(self.menubar, tearoff=0)
        self.dev_menu = tk.Menu(self.menubar, tearoff=1)

        self.menubar.add_cascade(label="Connection", menu=self.connect_menu)
        self.menubar.add_cascade(label="Mode", menu=self.mode_menu, state=tk.DISABLED)
        self.menubar.add_cascade(label="Autopilot", menu=self.autopilot_menu, state=tk.DISABLED)
        self.menubar.add_cascade(label="Camera", menu=self.cam_menu, state=tk.DISABLED)
        self.menubar.add_cascade(label="DEV", menu=self.dev_menu, state=tk.DISABLED)

        self.autopilot_menu.add_cascade(label="Navigation", menu=self.nav_menu)
        self.autopilot_menu.add_cascade(label="Setpoints", menu=self.sp_menu)

        self.connect_menu.add_command(label="Connect", command=self.connect)
        self.connect_menu.add_command(label="Close", command=self.close)

        self.mode_menu.add_command(label="Boot", command=self.boot)
        self.mode_menu.add_command(label="Set mode", command=self.show_set_mode_dialog)

        self.sp_menu.add_command(label="Set altitude", command=self.show_set_alt_dialog)
        self.sp_menu.add_command(label="Set speed", command=self.show_set_speed_dialog)

        self.nav_menu.add_command(label="Direct to", command=self.show_direct_to_dialog)

        self.cam_menu.add_command(label="Take image", command=self.cam_take_image)
        self.cam_menu.add_command(label="Pitch/yaw", command=self.show_pitchyaw_dialog)
        self.cam_menu.add_command(label="ROI", command=self.show_roi_dialog)
        self.cam_menu.add_command(label="ROI Clear", command=self.cam_roi_clear)
        self.cam_menu.add_command(label="Stow", command=self.cam_stow)

        self.dev_menu.add_command(label="PID", command=self.show_pid_tuner_dialog)

        self.connect_instance: Connect | None = None
        self.heartbeat = None

        self.update_status()

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
            self.map_widget.set_position(41.688306, -83.716114)
            self.map_widget.set_zoom(10)
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
                lat, lon, alt = MultiEntryDialog(self.root, ["Lat", "Lon", "Alt"]).result
                alt = float(alt) if alt is not None else alt
            except (TypeError, ValueError):
                return
            self.connect_instance.reposition(lat=lat, lon=lon, alt=alt)
            self.log("Direct to")

    def show_set_alt_dialog(self):
        if self.connect_instance is not None:
            try:
                alt = float(MultiEntryDialog(self.root, ["Altitude (ft)"]).result[0])
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
                lat, lon, alt = MultiEntryDialog(self.root, ["Lat", "Lon", "Alt"]).result
                alt = float(alt) if alt is not None else alt
            except (TypeError, ValueError):
                return
            self.connect_instance.gimbal_roi(lat=lat, lon=lon, alt=alt)
            self.log("ROI")

    def cam_take_image(self):
        if self.connect_instance is not None:
            self.connect_instance.img()
            self.log("Take image")

    def cam_roi_clear(self):
        if self.connect_instance is not None:
            self.connect_instance.gimbal_roi_clear()
            self.log("ROI Clear")

    def cam_stow(self):
        if self.connect_instance is not None:
            self.connect_instance.gimbal_pitchyaw(-180, 0)
            self.log("Stow")

    def show_pid_tuner_dialog(self):
        if self.connect_instance is not None:
            try:
                a, b, c, d = (value if value is not None else 0 for value in MultiEntryDialog(self.root, ["kp", "ti", "td", "sp"]).result)
            except TypeError:
                return
            self.connect_instance.pid(a, b, c, d)
            self.log("PID")

    def connect(self):
        try:
            target_id = int(MultiEntryDialog(self.root, ["Target ID"]).result[0])
        except ValueError:
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

    def boot(self):
        if self.connect_instance is not None:
            self.connect_instance.boot()
            self.log("Boot")

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
        self.log_text.insert(tk.END, message + '\n')
        self.log_text.see(tk.END)


def heartbeat(app: GCSUI) -> None:
    """Publish heartbeat message periodically in background."""
    try:
        while not stop.is_set():
            if app.connect_instance is not None:
                app.connect_instance.heartbeat()
            time.sleep(1)
    except KeyboardInterrupt:
        return


def listen(app: GCSUI) -> None:
    try:
        last_heartbeat = 0.0
        while not stop.is_set():
            if app.connect_instance is not None:
                if app.connect_instance.listen():
                    last_heartbeat = time.time()
                if time.time() - last_heartbeat > HEARTBEAT_TIMEOUT:
                    app.log(f"Heartbeat timeout from #{app.connect_instance.target}")
                    app.close(because_heartbeat=True)
            time.sleep(0)
    except KeyboardInterrupt:
        return


def main() -> None:
    app = GCSUI(tk.Tk())

    # Run the Tkinter mainloop in a separate thread
    heart = threading.Thread(target=heartbeat, args=(app,), daemon=True)
    heart.start()

    listener = threading.Thread(target=listen, args=(app,), daemon=True)
    listener.start()

    try:
        app.root.mainloop()
    except KeyboardInterrupt:
        pass


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
    def __init__(self, parent, title, text, **kwargs):
        self.text = text
        self.ok_pressed = False
        super().__init__(parent, title=title, **kwargs)

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


if __name__ == '__main__':
    main()
