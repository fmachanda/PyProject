import tkinter as tk

from gcs import Connect


class GCSUI:
    def __init__(self, root):
        self.root = root
        self.root.title('GCS Control')

        self.target_label = tk.Label(root, text='Target ID:')
        self.target_label.pack()

        self.target_entry = tk.Entry(root)
        self.target_entry.pack()

        self.connect_button = tk.Button(root, text='Connect', command=self.connect)
        self.connect_button.pack()

        self.boot_button = tk.Button(root, text='Boot', command=self.boot)
        self.boot_button.pack()

        self.close_button = tk.Button(root, text='Close', command=self.close)
        self.close_button.pack()

        self.command_label = tk.Label(root, text='Enter command(value):')
        self.command_label.pack()

        self.command_entry = tk.Entry(root)
        self.command_entry.pack()

        self.command_button = tk.Button(root, text='Send Command', command=self.command)
        self.command_button.pack()

        self.log_text = tk.Text(root, height=10, width=40)
        self.log_text.pack()

        self.connect_instance = None

    def connect(self):
        target_id = int(self.target_entry.get())

        if isinstance(target_id, int) and target_id<256:
            if self.connect_instance is None:
                self.connect_instance = Connect(target_id)
                if self.connect_instance.target is not None:
                    self.log(f'Connected to target #{target_id}')
                else:
                    self.connect_instance = None
                    self.log('Connection timeout')
            elif self.connect_instance.target != target_id:
                self.log(f'Close instance #{self.connect_instance.target} first')
            else:
                pass
        else:
            self.log('Error: ID must be UINT8')

    def command(self):
        cmd = str(self.command_entry.get())

        if self.connect_instance is not None:
            try:
                exec(f'self.connect_instance.{cmd}')
                self.log(f'Sent command \'{cmd}\'')
            except AttributeError:
                self.log(f'Command \'{cmd}\' invalid')
        else:
            self.log('Connect first')

    def boot(self):
        if self.connect_instance is not None:
            self.connect_instance.boot()
            self.log('Booting...')
        else:
            self.log('Connect first')

    def close(self):
        if self.connect_instance is not None:
            self.connect_instance.close()
            self.connect_instance = None
            self.log('Closing...')
        else:
            self.log('Connect first')

    def log(self, message):
        self.log_text.insert(tk.END, message + '\n')


if __name__ == '__main__':
    root = tk.Tk()
    app = GCSUI(root)
    root.mainloop()
