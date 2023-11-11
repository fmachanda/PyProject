import asyncio
import matplotlib.pyplot as plt
from collections import deque

async def imshow(img) -> None:
    await asyncio.sleep(0.1)
    plt.figure()
    plt.imshow(img)
    plt.show()

class Grapher:
    """Graph a deque of data in matplotlib."""
    def __init__(self, name: str = "Output", deque_len: int = 1500) -> None:
        """Inits the grpah."""
        self.fig,self.ax,self.line = Grapher.init_plot(name=name)
        self.data = deque(maxlen=deque_len)

    @staticmethod
    def init_plot(name: str = "Output") -> tuple:
        """Inits the plot."""
        plt.ion()  # Turn on interactive mode
        fig, ax = plt.subplots()
        line, = ax.plot([], [])
        ax.set_title(name)
        return fig, ax, line

    def add(self, new: float) -> None:
        """Add data to the deque (do this before running)."""
        try:
            self.data.append(new)
        except KeyboardInterrupt:
            self.close()
    
    def graph(self) -> None:
        """Graphs the deque of data (data added with \'Grapher.add()\')."""
        try:
            self.line.set_data(range(len(self.data)), self.data)
            self.ax.relim()
            self.ax.autoscale_view()
            plt.pause(0.001)
        except KeyboardInterrupt:
            self.close()

    def close(self) -> None:
        """Closes the graph."""
        plt.close(self.fig)
