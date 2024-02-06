import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.animation as animation

# Function to create a cube
def create_cube():
    # Define the vertices of the cube
    vertices = np.array([[-1, -1, -1],
                         [1, -1, -1 ],
                         [1, 1, -1],
                         [-1, 1, -1],
                         [-1, -1, 1],
                         [1, -1, 1 ],
                         [1, 1, 1],
                         [-1, 1, 1]])
    
    # Define the faces of the cube
    faces = [[vertices[i] for i in [0, 1, 2, 3]],
             [vertices[i] for i in [4, 5, 6, 7]], 
             [vertices[i] for i in [0, 3, 7, 4]], 
             [vertices[i] for i in [1, 2, 6, 5]], 
             [vertices[i] for i in [0, 1, 5, 4]],
             [vertices[i] for i in [2, 3, 7, 6]]]

    return faces

# Function to apply rotations
def rotate(faces, roll, pitch, yaw):
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(roll), -np.sin(roll)],
                   [0, np.sin(roll), np.cos(roll)]])
    
    Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                   [0, 1, 0],
                   [-np.sin(pitch), 0, np.cos(pitch)]])
    
    Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                   [np.sin(yaw), np.cos(yaw), 0],
                   [0, 0, 1]])

    R = np.dot(Rz, np.dot(Ry, Rx))

    return [[np.dot(vertex, R) for vertex in face] for face in faces]

# Animation update function
def update(num, ax, faces):
    ax.cla()
    # Setting the aspect ratio to be equal and plotting the 3D Cube
    ax.set_aspect('auto')

    # Read IMU data from file
    try:
       with open("rpy_data.txt", "r") as file:
            data = file.read().strip().split(',')
            pitch, roll, yaw = map(float, data)
    except Exception as e:
        print(f"Error: {e}")
        roll, pitch, yaw = 0, 0, 0

    # Rotate cube
    rotated_faces = rotate(faces, np.deg2rad(roll), np.deg2rad(pitch), np.deg2rad(yaw))

    # Plotting
    ax.add_collection3d(Poly3DCollection(rotated_faces, facecolors='cyan', linewidths=1, edgecolors='r', alpha=.25))

    # Setting the axes properties
    ax.set_xlim([-3, 3])
    ax.set_ylim([-3, 3])
    ax.set_zlim([-3, 3])

# Main function
def main():
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Creating the cube
    faces = create_cube()

    # Number of iterations (change this based on your data)
    num_frames = 200

    ani = animation.FuncAnimation(fig, update, frames=num_frames, fargs=(ax, faces), interval=50)

    plt.show()

if __name__ == "__main__":
    main()
