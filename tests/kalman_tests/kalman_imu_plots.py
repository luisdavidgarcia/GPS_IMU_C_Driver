import matplotlib.pyplot as plt
import time

# Initialize lists to store the roll, pitch, and yaw values
roll_values = []
pitch_values = []
yaw_values = []

plt.ion()  # Enable interactive mode
fig, ax = plt.subplots(3, 1, figsize=(10, 8))  # Create three subplots

def plot_data(roll, pitch, yaw):
    roll_values.append(roll)
    pitch_values.append(pitch)
    yaw_values.append(yaw)

    # Clear previous data
    if len(roll_values) > 50:  # Adjust this value as needed
        roll_values.pop(0)
        pitch_values.pop(0)
        yaw_values.pop(0)

    ax[0].cla()  # Clear the previous plot
    ax[1].cla()
    ax[2].cla()

    ax[0].plot(roll_values, label='Roll')
    ax[1].plot(pitch_values, label='Pitch')
    ax[2].plot(yaw_values, label='Yaw')

    ax[0].legend()
    ax[1].legend()
    ax[2].legend()

    plt.pause(0.1)  # Pause to update the plots

while True:
    try:
        with open("rpy_data.txt", "r") as file:
            data = file.read().strip()
            pitch, roll, yaw = map(float, data.split(','))  # Convert string data to float
            plot_data(roll, pitch, yaw)
    except IOError:
        print("File not accessible")
    except ValueError:
        print("Error in data format")
    time.sleep(1)  # Adjust the sleep time as needed
