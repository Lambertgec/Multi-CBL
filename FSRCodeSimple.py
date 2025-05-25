import inspect
if not hasattr(inspect, 'getargspec'):
    inspect.getargspec = inspect.getfullargspec

import pyfirmata
import time
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
from collections import deque

PORT = 'COM4'  # your port the arduino is connected to


board = pyfirmata.Arduino(PORT)
it = pyfirmata.util.Iterator(board)
it.start()

analog_input_0 = board.analog[0]
analog_input_1 = board.analog[1]
analog_input_0.enable_reporting()
analog_input_1.enable_reporting()

plt.ion()
fig, ax = plt.subplots()
plt.subplots_adjust(bottom=0.25)  

window_size = 100
data_0 = deque([0]*window_size, maxlen=window_size)
data_1 = deque([0]*window_size, maxlen=window_size)
line0, = ax.plot(data_0, label='A0', color='b')
line1, = ax.plot(data_1, label='A1', color='r')
ax.set_ylim(0, 1)
ax.set_title("Plot")
ax.legend()


stop_ax = plt.axes([0.4, 0.05, 0.2, 0.075])  
stop_button = Button(stop_ax, 'STOP', color='red', hovercolor='tomato')
should_stop = {'stop': False}

def stop(event):
    should_stop['stop'] = True

stop_button.on_clicked(stop)


voltage_text_a0 = ax.text(0.02, 0.90, 'A0: 0.00 V', transform=ax.transAxes, color='b', fontsize=12, fontweight='bold')
voltage_text_a1 = ax.text(0.02, 0.85, 'A1: 0.00 V', transform=ax.transAxes, color='r', fontsize=12, fontweight='bold')

try:
    while not should_stop['stop']:
        value0 = analog_input_0.read()
        value1 = analog_input_1.read()
        
        if value0 is not None:
            data_0.append(value0)
        else:
            data_0.append(0)
        if value1 is not None:
            data_1.append(value1)
        else:
            data_1.append(0)

        line0.set_ydata(data_0)
        line1.set_ydata(data_1)
        line0.set_xdata(range(len(data_0)))
        line1.set_xdata(range(len(data_1)))
        ax.relim()
        ax.autoscale_view()

        
        voltage_text_a0.set_text(f"A0: {data_0[-1]*5:.2f} V")
        voltage_text_a1.set_text(f"A1: {data_1[-1]*5:.2f} V")

        plt.draw()
        plt.pause(0.01)
except Exception as e:
    print("Error:", e)
finally:
    board.exit()
    plt.ioff()
    plt.show()
    print("Stopped!")
