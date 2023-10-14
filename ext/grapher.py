import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

class Grapher:

    def __init__(self, name:str='Output', deque_len:int=1500) -> None:
        self.fig,self.ax,self.line = Grapher.init_plot(name=name)
        self.data = deque(maxlen=deque_len)

    @staticmethod
    def init_plot(name:str='Output') -> tuple:
        plt.ion()  # Turn on interactive mode
        fig, ax = plt.subplots()
        line, = ax.plot([], [])
        ax.set_title(name)
        return fig, ax, line

    def add(self, new:float) -> None:
        try:
            self.data.append(new)
        except KeyboardInterrupt:
            self.close()
    
    def graph(self) -> None:
        try:
            self.line.set_data(range(len(self.data)), self.data)
            # try:
            self.ax.relim()
            # except:
            #     pass
            self.ax.autoscale_view()
            plt.pause(0.001)
        except KeyboardInterrupt:
            self.close()

    def close(self) -> None:
        plt.close(self.fig)