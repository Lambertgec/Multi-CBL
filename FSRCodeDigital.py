import time
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
from collections import deque
import random
import numpy as np

# window is ugly, will have to be fixed later

# this doesnt yet detect over time slow changes, for that it might make sense to compare every reading 
# to the one ~20 readings ago, if it is within a certain threshold, then posture is stable, otherwise unstable


wait_time = 1  # Time to wait between updates
pinsUsed = 8  # Number of simulated sensors
window_size = 100 # how many of the last readings are shown in the plot
max_deviation = 0.05 # Max allowed change per step (for random walk)
stability_seconds = 5 # how long readings have to be consistent before reporting posture as stable
probability_large_jump = 0.1 / pinsUsed # probability of value changing randomly (to simulate posture shift)
consistency_window = int(20 / wait_time)  # How many readings are checked for consistency
consistency_threshold = 0.65  # % of readings that must be consistent to be considered stable
mad_multiplier = 3  # For spike detection (see below)

plt.ion()
fig, ax = plt.subplots()
plt.subplots_adjust(bottom=0.25)

datas = [deque([random.uniform(0, 1)], maxlen=window_size) for _ in range(pinsUsed)] #deque to store sensor readings
colors = ['b', 'r', 'g', 'c', 'm', 'y', 'k', 'orange'] 
lines = []
for i in range(pinsUsed):
    lines.append(ax.plot(datas[i], label=f'A{i}', color=colors[i])[0])
ax.set_ylim(0, 1)
ax.set_title(f"Simulated Plot for the values read by the {pinsUsed} pressure sensors")
ax.legend()

stop_ax = plt.axes([0.4, 0.05, 0.2, 0.075])
stop_button = Button(stop_ax, 'STOP', color='red', hovercolor='tomato')
should_stop = {'stop': False}

def stop(event):
    should_stop['stop'] = True

stop_button.on_clicked(stop)

voltage_texts = [
    ax.text(0.02, -0.05 - i*0.05, f'A{i}: 0.00 V', transform=ax.transAxes, color=colors[i], fontsize=12, fontweight='bold')
    for i in range(pinsUsed)
]

# This is the timer checking how long the posture has been stable, 
#it is reset every time it is reported as unstable (so recent significant change)
last_unstable_time = time.time() 
stable_reported = False # Flag to track if stable posture has been reported (so we dont report it multiple times)
debounce_buffer = deque([True]*3, maxlen=3)  # Require 3 consecutive stable checks (to avoid false positives due to noise (spike due to momentary weight shifting))
enough_data = False # Flag to track if we have enough data for consistency checks

def mad(arr): #calculates the Median Absolute Deviation (MAD)
    med = np.median(arr)
    return np.median(np.abs(arr - med))

try:
    while not should_stop['stop']:
        # Simulate sensor readings
        for i in range(pinsUsed):
            prev = datas[i][-1]
            if random.random() < 1 - probability_large_jump: # Normal random walk (so change value with at most max_deviation)
                change = random.uniform(-max_deviation, max_deviation)
                new_val = min(max(prev + change, 0), 1)
            else: # Large random jump (to simulate sudden posture change)
                new_val = random.uniform(0, 1)
            datas[i].append(new_val) #new "reading" is added to the deque
            #update the plot with the new reading
            lines[i].set_ydata(datas[i])
            lines[i].set_xdata(range(len(datas[i])))

        if not enough_data and len(datas[0]) >= consistency_window: # Check if we have enough data for consistency checks (this is just to optimize)
            enough_data = True # Set flag to True if we have enough data
        ax.set_xlim(0, window_size - 1) # after all "readings" are added, shift the x-axis with the values

        for i in range(pinsUsed): # update the voltage text labels
            voltage_texts[i].set_text(f"A{i}: {datas[i][-1]*5:.2f} V")

        # --- Adaptive Consistency Window with Outlier-Resilient Baseline Tracking ---
        all_consistent = True # boolean to track if all sensors are consistent
        for i in range(pinsUsed):
            if enough_data: # check only if we have enough data
                window = np.array(list(datas[i])[-consistency_window:]) # Get the last `consistency_window` (so what we consider) readings
                baseline = np.median(window) # Use median as baseline to be resilient to outliers
                mad_val = mad(window) # Calculate the Median Absolute Deviation (MAD) of the window
                # consistency: % of readings within MAD*multiplier of baseline
                within_range = np.abs(window - baseline) < mad_multiplier * (mad_val + 1e-6)
                consistency_score = np.sum(within_range) / consistency_window

                # if the consistency score is below the threshold, then the sensor is inconsistent (there was a large shift in "readings")
                if consistency_score < consistency_threshold: 
                    all_consistent = False # as score is low, we set all_consistent to False as there is at least one sensor with inconsistent readings
                    print(f"Sensor A{i} inconsistent: {consistency_score:.2f} < {consistency_threshold}") # we print for debugging which values are inconsistent
                    break

        debounce_buffer.append(all_consistent) # keep track of the last few consistency checks
        print(f"all_consistent: {all_consistent}, debounce_buffer: {list(debounce_buffer)}") # also for debugging

        if all(debounce_buffer): # if all sensors are consistent, we check if the posture is stable for atleast `stability_seconds` seconds
            if not stable_reported and time.time() - last_unstable_time > stability_seconds: # if we didnt yet report stable, and enough time has passed 
                print(f"Stable posture detected for {stability_seconds} seconds!") # send signal that posture is stable (this is where we will light the LED)
                stable_reported = True # then communicate to avoid duplicate reports that we sent the signal already
        else: # otherwise, if any of the sensor had inconsistent readings
            last_unstable_time = time.time() #reset timer
            stable_reported = False # reset stable reported flag
            print("Timer reset")

        plt.draw()
        plt.pause(wait_time)
except Exception as e:
    print("Error:", e)
finally:
    plt.ioff()
    plt.show()
    print("Stopped!")