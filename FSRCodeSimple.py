import inspect
if not hasattr(inspect, 'getargspec'):
    inspect.getargspec = inspect.getfullargspec

import pyfirmata
import time
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
from collections import deque

board = pyfirmata.Arduino('COM4')
pyfirmata.util.Iterator(board).start()
pinsUsed  = 4 # can change this back to 2 until we wire more sensors, just use the first x pins on the board, max should be 8

inputs = [ [] for _ in range(pinsUsed) ]  # list of lists for 8 analog array inputs
for pin in range(pinsUsed):
    inputs[pin] = board.analog[pin] 
    inputs[pin].enable_reporting()

plt.ion()
fig, ax = plt.subplots()
plt.subplots_adjust(bottom=0.25)  

window_size = 100
datas = [deque([0]*window_size, maxlen=window_size) for _ in range(pinsUsed)] # deque to store the last 'window_size' values for each sensor
colors = ['b', 'r', 'g', 'c', 'm', 'y', 'k', 'orange'] # an array of colors for ease of assignment
lines = []
for i in range(pinsUsed):
    lines[i], = ax.plot(datas[i], label=f'A{i}', color = colors[i]) # set the data and color for each line
ax.set_ylim(0, 1)
ax.set_title("Plot for the values read by the 8 pressure sensors")
ax.legend()

stop_ax = plt.axes([0.4, 0.05, 0.2, 0.075])  
stop_button = Button(stop_ax, 'STOP', color='red', hovercolor='tomato')
should_stop = {'stop': False}

def stop(event):
    should_stop['stop'] = True

stop_button.on_clicked(stop)

# the array for the voltage text labels
voltage_texts = [
    ax.text(0.02, 0.90 - i*0.05, f'A{i}: 0.00 V', transform=ax.transAxes, color=colors[i], fontsize=12, fontweight='bold')
    for i in range(pinsUsed)
]

try:
    while not should_stop['stop']:

        # Read the values from sensors, depending on what they are update its data, then the lines
        values = [0.0] * pinsUsed
        for i in range(pinsUsed):
            values[i] = inputs[i].read()
            if values[i] is not None:
                datas[i].append(values[i])
            else:
                datas[i].append(0)            
            lines[i].set_ydata(datas[i])
            lines[i].set_xdata(range(len(datas[i])))    
        ax.relim()
        ax.autoscale_view()

        for i in range(pinsUsed): # update the voltage text labels
            voltage_texts[i].set_text(f"A{i}: {datas[i][-1]*5:.2f} V") 

        plt.draw()
        plt.pause(0.01)
except Exception as e:
    print("Error:", e)
finally:
    board.exit()
    plt.ioff()
    plt.show()
    print("Stopped!")
