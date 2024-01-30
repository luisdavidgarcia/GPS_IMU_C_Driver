# import numpy as np
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d.art3d import Line3D
# import matplotlib.animation as animation

# # Define a function to create a 3D cube
# def create_cube():
#     # Define the vertices of the cube
#     points = np.array([[-1, -1, -1],
#                        [1, -1, -1],
#                        [1, 1, -1],
#                        [-1, 1, -1],
#                        [-1, -1, 1],
#                        [1, -1, 1],
#                        [1, 1, 1],
#                        [-1, 1, 1]])
    
#     # Define the edges connecting the vertices
#     edges = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6),
#              (6, 7), (7, 4), (0, 4), (1, 5), (2, 6), (3, 7)]
    
#     return points, edges

# # Function to apply rotations to the cube
# def rotate_cube(points, roll, pitch, yaw):
#     Rx = np.array([[1, 0, 0],
#                    [0, np.cos(roll), -np.sin(roll)],
#                    [0, np.sin(roll), np.cos(roll)]])
    
#     Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)],
#                    [0, 1, 0],
#                    [-np.sin(pitch), 0, np.cos(pitch)]])
    
#     Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
#                    [np.sin(yaw), np.cos(yaw), 0],
#                    [0, 0, 1]])
    
#     # Apply rotations
#     R = np.dot(Rz, np.dot(Ry, Rx))
#     rotated_points = np.dot(points, R.T)
    
#     return rotated_points

# # Set up the figure and 3D axis
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')

# # Create the cube
# points, edges = create_cube()
# edge_lines = [Line3D([], [], [], color="b", marker="o") for _ in edges]
# for edge_line in edge_lines:
#     ax.add_line(edge_line)

# # Animation update function
# def update(frame, points, edges, edge_lines):
#     # Read IMU data from file (replace with your own reading mechanism)
#     try:
#         with open("rpy_data.txt", "r") as file:
#             data = file.read().strip().split(',')
#             pitch, roll, yaw = map(float, data)
#     except IOError:
#         print("File not accessible")
#         return
#     except ValueError:
#         print("Error in data format")
#         return

#     # Rotate cube
#     rotated_points = rotate_cube(points, roll, pitch, yaw)

#     # Update the edges of the cube
#     for edge_line, (start, end) in zip(edge_lines, edges):
#         edge_line.set_data([rotated_points[start, 0], rotated_points[end, 0]],
#                            [rotated_points[start, 1], rotated_points[end, 1]])
#         edge_line.set_3d_properties([rotated_points[start, 2], rotated_points[end, 2]])
    
#     # Set the limits of the axes
#     ax.set_xlim([-2, 2])
#     ax.set_ylim([-2, 2])
#     ax.set_zlim([-2, 2])
#     ax.set_xlabel('X')
#     ax.set_ylabel('Y')
#     ax.set_zlabel('Z')

#     return edge_lines

# # Create the animation
# ani = animation.FuncAnimation(fig, update, frames=6, fargs=(points, edges, edge_lines), interval=100, blit=False)

# plt.show()

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Line3D
import matplotlib.animation as animation

def create_cube():
    points = np.array([[-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1],
                       [-1, -1, 1], [1, -1, 1], [1, 1, 1], [-1, 1, 1]])
    edges = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4), (0, 4), (1, 5), (2, 6), (3, 7)]
    return points, edges

def rotate_cube(points, roll, pitch, yaw):
    Rx = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]])
    Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])
    Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
    R = np.dot(Rz, np.dot(Ry, Rx))
    rotated_points = np.dot(points, R.T)
    return rotated_points

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
points, edges = create_cube()
edge_lines = [Line3D([], [], [], color="b", marker="o") for _ in edges]
for edge_line in edge_lines:
    ax.add_line(edge_line)

def update(frame, points, edges, edge_lines):
    # Replace this section with real IMU and GPS data reading
    try:
        with open("rpy_data.txt", "r") as file:
            data = file.readline().strip().split(',')
            roll, pitch, yaw, lat, lon, alt = map(float, data)
    except (IOError, ValueError):
        print("Error reading data")
        return edge_lines

    rotated_points = rotate_cube(points, roll, pitch, yaw)
    
    # Transform latitude and longitude to a range suitable for visualization
    # Note: This is a simple linear transformation and won't accurately represent real-world distances
    scale = 0.01  # Adjust as needed for your visualization
    x_offset, y_offset, z_offset = lon * scale, lat * scale, alt * scale if alt else 0

    for edge_line, (start, end) in zip(edge_lines, edges):
        edge_line.set_data([rotated_points[start, 0] + x_offset, rotated_points[end, 0] + x_offset],
                           [rotated_points[start, 1] + y_offset, rotated_points[end, 1] + y_offset])
        edge_line.set_3d_properties([rotated_points[start, 2] + z_offset, rotated_points[end, 2] + z_offset])

    ax.set_xlim([-2 + x_offset, 2 + x_offset])
    ax.set_ylim([-2 + y_offset, 2 + y_offset])
    ax.set_zlim([-2 + z_offset, 2 + z_offset])
    ax.set_xlabel('Longitude')
    ax.set_ylabel('Latitude')
    ax.set_zlabel('Altitude' if alt else 'Z')

    return edge_lines

ani = animation.FuncAnimation(fig, update, frames=6, fargs=(points, edges, edge_lines), interval=100, blit=False)
plt.show()
