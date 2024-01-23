import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Initialize serial port
ser = serial.Serial('/dev/ttyAMA1', 115200, timeout=1)

# Prepare matplotlib plot
fig, (ax1, ax2, ax3) = plt.subplots(3, 1)
x_data, roll_data, pitch_data, yaw_data = [], [], [], []

# Function to update plot
def update(frame):
    line = ser.readline().decode('utf-8').strip()
    if line:
        data = line.split(',')
        try:
            roll, pitch, yaw = map(float, data)
            x_data.append(frame)
            roll_data.append(roll)
            pitch_data.append(pitch)
            yaw_data.append(yaw)

            ax1.clear()
            ax1.plot(x_data, roll_data)
            ax1.set_title('Roll')

            ax2.clear()
            ax2.plot(x_data, pitch_data)
            ax2.set_title('Pitch')

            ax3.clear()
            ax3.plot(x_data, yaw_data)
            ax3.set_title('Yaw')
        except ValueError:
            pass

# Initialize animation
ani = FuncAnimation(fig, update, interval=1000)
plt.show()
