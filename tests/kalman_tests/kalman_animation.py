import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
import numpy as np

# Define a function to create a 3D cube
def create_cube():
    # Define the vertices of the cube
    points = np.array([[-1, -1, -1],
                       [1, -1, -1 ],
                       [1, 1, -1],
                       [-1, 1, -1],
                       [-1, -1, 1],
                       [1, -1, 1 ],
                       [1, 1, 1],
                       [-1, 1, 1]])
    
    # Define the edges connecting the vertices
    edges = [[points[i], points[j]] for i in range(len(points)) for j in range(i+1, len(points))]
    
    return points, edges

# Function to apply rotations to the cube
def rotate_cube(points, roll, pitch, yaw):
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(roll), -np.sin(roll)],
                   [0, np.sin(roll), np.cos(roll)]])
    
    Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                   [0, 1, 0],
                   [-np.sin(pitch), 0, np.cos(pitch)]])
    
    Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                   [np.sin(yaw), np.cos(yaw), 0],
                   [0, 0, 1]])
    
    # Apply rotations
    R = np.dot(Rz, np.dot(Ry, Rx))
    rotated_points = np.dot(points, R.T)
    
    return rotated_points

# Set up the figure and 3D axis
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Create the cube
points, edges = create_cube()
edge_lines = [ax.plot(edge[0][0:1], edge[0][1:2], edge[0][2:3])[0] for edge in edges]

# Animation update function
def update(frame):
    global points
    # Read IMU data from file (replace with your own reading mechanism)
    try:
        with open("data.txt", "r") as file:
            data = file.read().strip().split(',')
            roll, pitch, yaw = map(float, data)
    except IOError:
        print("File not accessible")
        return
    
    # Clear axes
    ax.cla()
    ax.set_xlim([-2, 2])
    ax.set_ylim([-2, 2])
    ax.set_zlim([-2, 2])
    
    # Rotate cube
    rotated_points = rotate_cube(points, roll, pitch, yaw)
    
    # Update the edges of the cube
    for edge_line, edge in zip(edge_lines, edges):
        edge_line.set_data([rotated_points[edge[0]][0], rotated_points[edge[1]][0]], 
                           [rotated_points[edge[0]][1], rotated_points[edge[1]][1]])
        edge_line.set_3d_properties([rotated_points[edge[0]][2], rotated_points[edge[1]][2]])
        
        ax.add_line(edge_line)
    
    return edge_lines

# Create the animation
ani = animation.FuncAnimation(fig, update, frames=6, interval=1000, blit=False)

plt.show()
