import serial
import time
import matplotlib.pyplot as plt
import numpy as np
import csv
from collections import deque

# Serial config
port = '/dev/tty.usbmodem1101'  # Update if needed
baudrate = 9600
duration = 300
window_size = 20  # seconds in live plot

# Deques for sliding window
timestamps = deque()
setpoints = deque()
temperatures = deque()
outputs = deque()

Kp = Ki = Kd = error = None  # Placeholders

# Try opening serial
try:
    ser = serial.Serial(port, baudrate, timeout=1)
except serial.SerialException as e:
    print(f"Serial error: {e}")
    exit()

start_time = time.time()
print("Starting live PID data collection...")

plt.ion()
fig, ax = plt.subplots(figsize=(10, 6))

while True:
    if time.time() - start_time > duration:
        break

    line = ser.readline().decode('utf-8', errors='ignore').strip()
    if not line:
        continue

    print("Received:", repr(line))

    if line.startswith("SetTemp:"):
        parts = line.split()
        if len(parts) >= 3:  # Only 3 expected fields
            try:
                set_temp = float(parts[0].split(":")[1])
                real_temp = float(parts[1].split(":")[1])
                output = float(parts[2].split(":")[1])

                current_time = time.time() - start_time

                # Append data
                timestamps.append(current_time)
                setpoints.append(set_temp)
                temperatures.append(real_temp)
                outputs.append(output)

                # Keep window size
                while timestamps and (current_time - timestamps[0] > window_size):
                    timestamps.popleft()
                    setpoints.popleft()
                    temperatures.popleft()
                    outputs.popleft()

                # Plot data
                ax.clear()
                ax.plot(timestamps, temperatures, label='Measured Temperature')
                ax.plot(timestamps, setpoints, label='Setpoint', linestyle='--')
                ax.plot(timestamps, outputs, label='Output', linestyle=':')
                ax.set_xlabel('Time (s)')
                ax.set_ylabel('Temperature / Output')
                ax.set_title('Live PID Response (last 20s)')
                ax.legend()
                ax.grid(True)
                plt.tight_layout()
                plt.pause(0.01)

            except Exception as e:
                print("Parse error:", e)
        else:
            print("Unexpected line format:", parts)

ser.close()
print("Data collection finished.")

# Save data
with open('pid_data_simple.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(['Time (s)', 'Setpoint (°C)', 'Temperature (°C)', 'Output'])
    for i in range(len(timestamps)):
        writer.writerow([timestamps[i], setpoints[i], temperatures[i], outputs[i]])

print("Saved to pid_data_simple.csv")
plt.ioff()
plt.show()