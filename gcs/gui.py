# import asyncio
import re
import threading
import time
import tkinter as tk

import tkintermapview as tkmap

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

        self.target_label = tk.Label(root, text="Target ID:")
        self.target_label.pack()

        self.target_entry = tk.Entry(root)
        self.target_entry.pack()

        self.connect_button = tk.Button(root, text="Connect", command=self.connect)
        self.connect_button.pack()

        self.boot_button = tk.Button(root, text="Boot", command=self.boot)
        self.boot_button.pack()

        self.close_button = tk.Button(root, text="Close", command=self.close)
        self.close_button.pack()

        self.command_label = tk.Label(root, text="Enter command(value):")
        self.command_label.pack()

        self.command_entry = tk.Entry(root)
        self.command_entry.pack()

        self.command_button = tk.Button(root, text="Send Command", command=self.command)
        self.command_button.pack()

        self.log_text = tk.Text(root, height=10, width=40)
        self.log_text.pack()

        self.map_button = tk.Button(root, text="Open Map", command=self.open_map)
        self.map_button.pack()

        self.connect_instance: Connect | None = None
        self.heartbeat = None

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

    def connect(self):
        try:
            target_id = int(self.target_entry.get())
        except ValueError:
            self.log("Error: ID must be UINT8")
            return

        if isinstance(target_id, int) and target_id<256:
            if self.connect_instance is None:
                self.connect_instance = Connect(target_id)
                if self.connect_instance.target is not None:
                    self.log(f"Connected to target #{target_id}")
                else:
                    self.connect_instance = None
                    self.log("Connection timeout")
            elif self.connect_instance.target != target_id:
                self.log(f"Close instance #{self.connect_instance.target} first")
            else:
                pass
        else:
            self.log("Error: ID must be UINT8")

    def command(self):
        cmd = str(self.command_entry.get())
        try:
            self.connect_instance.map_pos = list(self.map_marker.position)
        except AttributeError:
            self.connect_instance.map_pos = [0.0, 0.0]

        if self.connect_instance is not None:
            allowed_commands = {
                'boot': self.connect_instance.boot,
                'set_mode': self.connect_instance.set_mode,
                'set_alt': self.connect_instance.set_alt,
                'set_speed': self.connect_instance.set_speed,
                'reposition': self.connect_instance.reposition,
                'f_takeoff': self.connect_instance.f_takeoff,
                'v_takeoff': self.connect_instance.v_takeoff,
                'f_land': self.connect_instance.f_land,
                'v_land': self.connect_instance.v_land,
                'pid': self.connect_instance.pid,
                'img': self.connect_instance.img,
            }

            pattern = re.compile(r'(\w+)\((.*?)\)')
            match_ = pattern.match(cmd)

            if match_:
                function_name = match_.group(1)
                arguments_str = match_.group(2)

                arguments = [arg.strip() for arg in arguments_str.split(',')]
                arguments = [arg for arg in arguments if arg and not arg.isspace()]

                method_to_call = allowed_commands.get(function_name)

                if method_to_call:
                    result = method_to_call(*arguments)
                    self.log(f"Executed command '{cmd}' with result: {result}")
                else:
                    print(f"Invalid function name: {function_name}")
            else:
                print(f"Invalid command format: {cmd}")
        else:
            self.log("Connect first")

    def boot(self):
        if self.connect_instance is not None:
            self.connect_instance.boot()
            self.log("Booting...")
        else:
            self.log("Connect first")

    def close(self, because_heartbeat: bool = False):
        if self.connect_instance is not None:
            self.connect_instance.close(because_heartbeat)
            self.connect_instance = None
            self.log("Closing...")
        else:
            self.log("Connect first")

    def window_close(self):
        self.close()
        stop.set()
        self.root.destroy()

    def log(self, message):
        self.log_text.insert(tk.END, message + '\n')


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


if __name__ == '__main__':
    main()
