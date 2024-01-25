import matplotlib.pyplot as plt
import time

# Initialize lists for each metric
# Raw GPS data
latitudes, longitudes, altitudes, speeds_n, speeds_e, speeds_d = [], [], [], [], [], []

# Filtered data
latitudes_filtered, longitudes_filtered, altitudes_filtered = [], [], []
speeds_n_filtered, speeds_e_filtered, speeds_d_filtered = [], [], []

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

    latitudes_filtered.append(data[6])
    longitudes_filtered.append(data[7])
    altitudes_filtered.append(data[8])
    speeds_n_filtered.append(data[9])
    speeds_e_filtered.append(data[10])
    speeds_d_filtered.append(data[11])

    rolls.append(data[12])
    pitches.append(data[13])
    yaws.append(data[14])

def plot_data():
    # Clear previous data and plot new data for each metric
    for i, (data, data_filtered, title) in enumerate(zip(
        [latitudes, longitudes, altitudes, speeds_n, speeds_e, speeds_d],
        [latitudes_filtered, longitudes_filtered, altitudes_filtered, speeds_n_filtered, speeds_e_filtered, speeds_d_filtered],
        ["Latitude", "Longitude", "Altitude", "Speed North", "Speed East", "Speed Down"]
    )):
        axs[i].cla()
        axs[i].plot(data, label=f'{title} Raw')
        axs[i].plot(data_filtered, label=f'{title} Filtered')
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
        with open("data.txt", "r") as file:
            for line in file:
                data = list(map(float, line.strip().split(',')))
                add_data_to_lists(data)
            plot_data()
    except IOError:
        print("File not accessible")
    except ValueError:
        print("Error in data format")
    time.sleep(1)  # Adjust the sleep time as needed
