# import asyncio
import threading
import time
import tkinter as tk

from gcs import Connect

stop = threading.Event()

HEARTBEAT_TIMEOUT = 2.0


class GCSUI:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("GCS Control")
        self.root.protocol("WM_DELETE_WINDOW", self.window_close)

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

        self.connect_instance: Connect | None = None
        self.heartbeat = None

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

        if self.connect_instance is not None:
            try:
                assert ';' not in cmd and '=' not in cmd
                exec(f'self.connect_instance.{cmd}')
                self.log(f"Sent command '{cmd}'")
            except AssertionError:
                self.log(f"Command '{cmd}' invalid")
            except AttributeError:
                self.log(f"Command '{cmd}' invalid")
            except SyntaxError:
                self.log(f"Command '{cmd}' invalid")
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