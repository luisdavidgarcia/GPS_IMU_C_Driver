import matplotlib.pyplot as plt
import time

# Initialize lists for each metric
# Raw GPS data
latitudes, longitudes, altitudes, speeds_n, speeds_e, speeds_d = [], [], [], [], [], []

# Roll, Pitch, Yaw
rolls, pitches, yaws = [], [], []

plt.ion()
fig, axs = plt.subplots(9, 1, figsize=(15, 9))  # Adjusted figure size to be wider and less tall

def add_data_to_lists(data):
    latitudes.append(data[0])
    longitudes.append(data[1])
    altitudes.append(data[2])
    speeds_n.append(data[3])
    speeds_e.append(data[4])
    speeds_d.append(data[5])
    rolls.append(data[6])
    pitches.append(data[7])
    yaws.append(data[8])

def plot_data():
    # Clear previous data and plot new data for each metric
    for i, (data, title) in enumerate(zip(
        [latitudes, longitudes, altitudes, speeds_n, speeds_e, speeds_d],
        ["Latitude", "Longitude", "Altitude", "Speed North", "Speed East", "Speed Down"]
    )):
        axs[i].cla()
        axs[i].plot(data, label=f'{title} Raw')
        axs[i].legend()

    axs[6].cla()
    axs[6].plot(rolls, label='Roll')
    axs[6].legend()

    axs[7].cla()
    axs[7].plot(pitches, label='Pitch')
    axs[7].legend()

    axs[8].cla()
    axs[8].plot(yaws, label='Yaw')
    axs[8].legend()

    plt.tight_layout()  # Adjust layout
    plt.pause(0.1)

while True:
    try:
        with open("rpy_data.txt", "r") as file:
            for line in file:
                data = list(map(float, line.strip().split(',')))
                add_data_to_lists(data)
            plot_data()
    except IOError:
        print("File not accessible")
    except ValueError:
        print("Error in data format")
    time.sleep(1)  # Adjust the sleep time as needed
