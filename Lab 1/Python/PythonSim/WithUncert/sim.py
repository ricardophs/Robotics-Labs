import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd
from time import sleep
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection
from matplotlib.patches import Rectangle
import mpl_toolkits.mplot3d.art3d as art3d
from IPython import display
import sys

# Function that draws one lego given the dimensions
def get_lego_edges(cube_definition):
    cube_definition_array = [
        np.array(list(item))
        for item in cube_definition
    ]

    points = []
    points += cube_definition_array
    vectors = [
        cube_definition_array[1] - cube_definition_array[0],
        cube_definition_array[2] - cube_definition_array[0],
        cube_definition_array[3] - cube_definition_array[0]
    ]

    points += [cube_definition_array[0] + vectors[0] + vectors[1]]
    points += [cube_definition_array[0] + vectors[0] + vectors[2]]
    points += [cube_definition_array[0] + vectors[1] + vectors[2]]
    points += [cube_definition_array[0] + vectors[0] + vectors[1] + vectors[2]]

    points = np.array(points)

    edges = [
        [points[0], points[3], points[5], points[1]],
        [points[1], points[5], points[7], points[4]],
        [points[4], points[2], points[6], points[7]],
        [points[2], points[6], points[3], points[0]],
        [points[0], points[2], points[4], points[1]],
        [points[3], points[6], points[7], points[5]]
    ]

    return np.array(edges)

# Axis limits
x_min = 0
x_max = 0.4
y_min = -0.25
y_max = 0.15
z_min = 0
z_max = 0.3

# Dimensions of the lego piece and velcro
lego_height = 0.031
lego_side = 0.032
velcro_height = 0.004
velcro_side = 0.015

# Defines and extra space between the pieces
extra_space = 0.001

origin_center_x = 0.1075
origin_center_y = -0.180

target_corner_x = 0.194
target_corner_y = 0.034

# Get trajectory points from text file
with open("traj.txt") as f:
    lines = f.read().splitlines()
N_points = int(len(lines))
points = []
for i in range(N_points):
    points.append(np.fromstring(lines[i], dtype=float, sep=' '))
points = np.array(points)

# Start position
start_position_x = points[0][0]
start_position_y = points[0][1]
start_position_z = points[0][2]

# Start orientation
l = 0.04
start_ori_x = np.array([start_position_x, start_position_y+l, start_position_z])
start_ori_y = np.array([start_position_x+l*np.cos(0.757), start_position_y, start_position_z+l*np.sin(0.757)])
start_ori_z = np.array([start_position_x+l*np.sin(0.757), start_position_y, start_position_z-l*np.cos(0.757)])

# Get poses indices from text file
with open("indices.txt") as f:
    lines = f.read().splitlines()
N_stops = int(len(lines))
stops = []
for i in range(N_stops):
    stops.append(int(np.fromstring(lines[i], dtype=int, sep=' ')))
stops = np.array(stops)

# Get gripper orientations from text file
with open("ori.txt") as f:
    lines = f.read().splitlines()
ori = []
for i in range(N_points):
    ori.append(np.fromstring(lines[i], dtype=float, sep=' '))
ori = np.array(ori)

# Figure creation
fig = plt.figure(figsize=(15,7.5))
# Use latex letter type
plt.rc('text', usetex=True)
plt.rc('font', family='serif')

# Pack coordinates into a data frame
coord = pd.DataFrame(points[:,0:3], columns=["x","y","z"])

# Define 3d axis
ax = plt.axes(projection='3d')

# Plotting world axis
ax.plot([0, 0], [y_min, y_max], [0, 0], linestyle='--', color='black', alpha=0.5)
ax.plot([x_min, x_max], [0, 0], [0, 0], linestyle='--', color='black', alpha=0.5)
ax.plot([0, 0], [0, 0], [z_min, z_max], linestyle='--', color='black', alpha=0.5)

# Initial view
ax.view_init(65, -20)

# Axis labels
ax.set_xlabel(r'$x$')
ax.set_ylabel(r'$y$')
ax.set_zlabel(r'$z$')

# Plot Title
plt.title("Niryo One Simulation")

# Plot Limits
ax.set_xlim3d(x_min, x_max)
ax.set_ylim3d(y_min, y_max)
ax.set_zlim3d(z_min, z_max)

# Drawing Lego Tower

# Positions of one corner of the bottom lego
corner_x = origin_center_x-lego_side/2
corner_y = origin_center_y-lego_side/2
corner_z = 0
cube_definition = [(corner_x, corner_y, corner_z), (corner_x+lego_side, corner_y, corner_z), (corner_x, corner_y+lego_side, corner_z), (corner_x, corner_y, corner_z+lego_height)]
edges_c1 = get_lego_edges(cube_definition)

# Origin rectangle
p = Rectangle((corner_x-extra_space, corner_y-extra_space), lego_side+2*extra_space, lego_side+2*extra_space, fill=False, edgecolor='black')
ax.add_patch(p)
art3d.pathpatch_2d_to_3d(p, z=0, zdir='z')

# Target rectangle
p = Rectangle((target_corner_x, target_corner_y), 4*extra_space+2*lego_side, 4*extra_space+2*lego_side, fill=False, edgecolor='black')
ax.add_patch(p)
art3d.pathpatch_2d_to_3d(p, z=0, zdir='z')

# Velcro
corner_x = origin_center_x-velcro_side/2
corner_y = origin_center_y-lego_side/2
corner_z = lego_height
cube_definition = [(corner_x, corner_y, corner_z), (corner_x+velcro_side, corner_y, corner_z), (corner_x, corner_y+lego_side, corner_z), (corner_x, corner_y, corner_z+velcro_height)]
edges_v1 = get_lego_edges(cube_definition)

# Second lego in the tower
corner_x = origin_center_x-lego_side/2
corner_y = origin_center_y-lego_side/2
corner_z = (lego_height + velcro_height)
cube_definition = [(corner_x, corner_y, corner_z), (corner_x+lego_side, corner_y, corner_z), (corner_x, corner_y+lego_side, corner_z), (corner_x, corner_y, corner_z+lego_height)]
edges_c2 = get_lego_edges(cube_definition)

# Velcro
corner_x = origin_center_x-lego_side/2
corner_y = origin_center_y-velcro_side/2
corner_z = (2*lego_height + velcro_height)
cube_definition = [(corner_x, corner_y, corner_z), (corner_x+lego_side, corner_y, corner_z), (corner_x, corner_y+velcro_side, corner_z), (corner_x, corner_y, corner_z+velcro_height)]
edges_v2 = get_lego_edges(cube_definition)

# Third lego in the tower
corner_x = origin_center_x-lego_side/2
corner_y = origin_center_y-lego_side/2
corner_z = 2*(lego_height + velcro_height)
cube_definition = [(corner_x, corner_y, corner_z), (corner_x+lego_side, corner_y, corner_z), (corner_x, corner_y+lego_side, corner_z), (corner_x, corner_y, corner_z+lego_height)]
edges_c3 = get_lego_edges(cube_definition)

# Velcro
corner_x = origin_center_x-velcro_side/2
corner_y = origin_center_y-lego_side/2
corner_z = (3*lego_height + 2*velcro_height)
cube_definition = [(corner_x, corner_y, corner_z), (corner_x+velcro_side, corner_y, corner_z), (corner_x, corner_y+lego_side, corner_z), (corner_x, corner_y, corner_z+velcro_height)]
edges_v3 = get_lego_edges(cube_definition)

# Fourth lego in the tower
corner_x = origin_center_x-lego_side/2
corner_y = origin_center_y-lego_side/2
corner_z = 3*(lego_height + velcro_height)
cube_definition = [(corner_x, corner_y, corner_z), (corner_x+lego_side, corner_y, corner_z), (corner_x, corner_y+lego_side, corner_z), (corner_x, corner_y, corner_z+lego_height)]
edges_c4 = get_lego_edges(cube_definition)

# Velcro
corner_x = origin_center_x-lego_side/2
corner_y = origin_center_y-velcro_side/2
corner_z = (4*lego_height + 3*velcro_height)
cube_definition = [(corner_x, corner_y, corner_z), (corner_x+lego_side, corner_y, corner_z), (corner_x, corner_y+velcro_side, corner_z), (corner_x, corner_y, corner_z+velcro_height)]
edges_v4 = get_lego_edges(cube_definition)

# Ading all the legos and velcros
faces = Poly3DCollection(edges_c1, linewidths=1, edgecolors='k')
faces.set_facecolor((0,0,1,1))
ax.add_collection3d(faces)

faces = Poly3DCollection(edges_c2, linewidths=1, edgecolors='k')
faces.set_facecolor((0,0,1,1))
ax.add_collection3d(faces)

faces = Poly3DCollection(edges_c3, linewidths=1, edgecolors='k')
faces.set_facecolor((0,0,1,1))
ax.add_collection3d(faces)

faces = Poly3DCollection(edges_c4, linewidths=1, edgecolors='k')
faces.set_facecolor((0,0,1,1))
ax.add_collection3d(faces)

faces = Poly3DCollection(edges_v1, linewidths=1, edgecolors='k')
faces.set_facecolor((0,0,0,1))
ax.add_collection3d(faces)

faces = Poly3DCollection(edges_v2, linewidths=1, edgecolors='k')
faces.set_facecolor((0,0,0,1))
ax.add_collection3d(faces)

faces = Poly3DCollection(edges_v3, linewidths=1, edgecolors='k')
faces.set_facecolor((0,0,0,1))
ax.add_collection3d(faces)

faces = Poly3DCollection(edges_v4, linewidths=1, edgecolors='k')
faces.set_facecolor((0,0,0,1))
ax.add_collection3d(faces)

line, = ax.plot([], [], lw=2, color='red')
# Point that represent the gripper
sc = ax.scatter(start_position_x, start_position_y, start_position_z, c='black', alpha=1)

# x axis of the gripper
Qx = ax.quiver3D(
        start_position_x, start_position_y, start_position_z,
        start_ori_x[0] - start_position_x, start_ori_x[1] - start_position_y, start_ori_x[2] - start_position_z,
        color = 'red', alpha = 1, lw = 2, arrow_length_ratio=0.3
    )
# y axis of the gripper
Qy = ax.quiver3D(
        start_position_x, start_position_y, start_position_z,
        start_ori_y[0] - start_position_x, start_ori_y[1] - start_position_y, start_ori_y[2] - start_position_z,
        color = 'green', alpha = 1, lw = 2, arrow_length_ratio=0.3
    )
# z axis of the gripper
Qz = ax.quiver3D(
        start_position_x, start_position_y, start_position_z,
        start_ori_z[0] - start_position_x, start_ori_z[1] - start_position_y, start_ori_z[2] - start_position_z,
        color = 'blue', alpha = 1, lw = 2, arrow_length_ratio=0.3
    )

# Main function that updates the animation
def update(i):
    # In between the poses that where originaly defines, the gripper makes a small stop
    if i in stops:
        sleep(0.5)

    # Auxiliary coordinates
    x = float(coord.x.values[i:i+1])
    y = float(coord.y.values[i:i+1])
    z = float(coord.z.values[i:i+1])
    line.set_data_3d([], [], [])

    # Depending on where we are in the simulation, we need to update the lego and the velcro positions
    if (i >= stops[3]) and (i <= stops[6]):
        # LEGO 4
        corner_x = float(origin_center_x-lego_side/2+(x-coord.x.values[stops[3]:stops[3]+1]))
        corner_y = float(origin_center_y-lego_side/2+(y-coord.y.values[stops[3]:stops[3]+1]))
        corner_z = float(z - extra_space - (lego_height + velcro_height))
        cube_definition = [(corner_x, corner_y, corner_z), (corner_x+lego_side, corner_y, corner_z), (corner_x, corner_y+lego_side, corner_z), (corner_x, corner_y, corner_z+lego_height)]
        edges = get_lego_edges(cube_definition)
        ax.collections[3].set_verts(edges)
        # Velcro
        corner_x = float(origin_center_x-lego_side/2+(x-coord.x.values[stops[3]:stops[3]+1]))
        corner_y = float(origin_center_y-velcro_side/2+(y-coord.y.values[stops[3]:stops[3]+1]))
        corner_z = float(z - extra_space - (velcro_height))
        cube_definition = [(corner_x, corner_y, corner_z), (corner_x+lego_side, corner_y, corner_z), (corner_x, corner_y+velcro_side, corner_z), (corner_x, corner_y, corner_z+velcro_height)]
        edges = get_lego_edges(cube_definition)
        ax.collections[7].set_verts(edges)
    if (i >= stops[9]) and (i <= stops[12]):
        # LEGO 3
        corner_x = float(origin_center_x-lego_side/2+(x-coord.x.values[stops[9]:stops[9]+1]))
        corner_y = float(origin_center_y-lego_side/2+(y-coord.y.values[stops[9]:stops[9]+1]))
        corner_z = float(z - extra_space - (lego_height + velcro_height))
        cube_definition = [(corner_x, corner_y, corner_z), (corner_x+lego_side, corner_y, corner_z), (corner_x, corner_y+lego_side, corner_z), (corner_x, corner_y, corner_z+lego_height)]
        edges = get_lego_edges(cube_definition)
        ax.collections[2].set_verts(edges)
        # Velcro
        corner_x = float(origin_center_x-velcro_side/2+(x-coord.x.values[stops[9]:stops[9]+1]))
        corner_y = float(origin_center_y-lego_side/2+(y-coord.y.values[stops[9]:stops[9]+1]))
        corner_z = float(z - extra_space - (velcro_height))
        cube_definition = [(corner_x, corner_y, corner_z), (corner_x+velcro_side, corner_y, corner_z), (corner_x, corner_y+lego_side, corner_z), (corner_x, corner_y, corner_z+velcro_height)]
        edges = get_lego_edges(cube_definition)
        ax.collections[6].set_verts(edges)
    if (i >= stops[15]) and (i <= stops[18]):
        # LEGO 2
        corner_x = float(origin_center_x-lego_side/2+(x-coord.x.values[stops[15]:stops[15]+1]))
        corner_y = float(origin_center_y-lego_side/2+(y-coord.y.values[stops[15]:stops[15]+1]))
        corner_z = float(z - extra_space - (lego_height + velcro_height))
        cube_definition = [(corner_x, corner_y, corner_z), (corner_x+lego_side, corner_y, corner_z), (corner_x, corner_y+lego_side, corner_z), (corner_x, corner_y, corner_z+lego_height)]
        edges = get_lego_edges(cube_definition)
        ax.collections[1].set_verts(edges)
        # Velcro
        corner_x = float(origin_center_x-lego_side/2+(x-coord.x.values[stops[15]:stops[15]+1]))
        corner_y = float(origin_center_y-velcro_side/2+(y-coord.y.values[stops[15]:stops[15]+1]))
        corner_z = float(z - extra_space - (velcro_height))
        cube_definition = [(corner_x, corner_y, corner_z), (corner_x+lego_side, corner_y, corner_z), (corner_x, corner_y+velcro_side, corner_z), (corner_x, corner_y, corner_z+velcro_height)]
        edges = get_lego_edges(cube_definition)
        ax.collections[5].set_verts(edges)
    if (i >= stops[21]) and (i <= stops[24]):
        # LEGO 1
        corner_x = float(origin_center_x-lego_side/2+(x-coord.x.values[stops[21]:stops[21]+1]))
        corner_y = float(origin_center_y-lego_side/2+(y-coord.y.values[stops[21]:stops[21]+1]))
        corner_z = float(z - extra_space - (lego_height + velcro_height))
        cube_definition = [(corner_x, corner_y, corner_z), (corner_x+lego_side, corner_y, corner_z), (corner_x, corner_y+lego_side, corner_z), (corner_x, corner_y, corner_z+lego_height)]
        edges = get_lego_edges(cube_definition)
        ax.collections[0].set_verts(edges)
        # Velcro
        corner_x = float(origin_center_x-velcro_side/2+(x-coord.x.values[stops[21]:stops[21]+1]))
        corner_y = float(origin_center_y-lego_side/2+(coord.y.values[i:i+1]-coord.y.values[stops[21]:stops[21]+1]))
        corner_z = float(z - extra_space - (velcro_height))
        cube_definition = [(corner_x, corner_y, corner_z), (corner_x+velcro_side, corner_y, corner_z), (corner_x, corner_y+lego_side, corner_z), (corner_x, corner_y, corner_z+velcro_height)]
        edges = get_lego_edges(cube_definition)
        ax.collections[4].set_verts(edges)

    # Removing the axis of the gripper and redrawing it
    plt.gca().collections[9].remove()
    plt.gca().collections[9].remove()
    plt.gca().collections[9].remove()

    Qx = ax.quiver3D(
            x, y, z,
            ori[i][0] - x, ori[i][1] - y, ori[i][2] - z,
            color = 'red', alpha = 1, lw = 2, arrow_length_ratio=0.3
        )
    Qy = ax.quiver3D(
            x, y, z,
            ori[i][3] - x, ori[i][4] - y, ori[i][5] - z,
            color = 'green', alpha = 1, lw = 2, arrow_length_ratio=0.3
        )
    Qz = ax.quiver3D(
            x, y, z,
            ori[i][6] - x, ori[i][7] - y, ori[i][8] - z,
            color = 'blue', alpha = 1, lw = 2, arrow_length_ratio=0.3
        )

    # Update the gripper coordinates
    sc._offsets3d = (coord.x.values[i:i+1], coord.y.values[i:i+1], coord.z.values[i:i+1])
    
    return line

# Animation definition
anim = FuncAnimation(fig, update, frames=N_points, interval=10, blit=False, repeat= False)

# Depending or the arguments when running the script, we can show the animation or we can save it to a mp4 file
if len(sys.argv) == 2 and str(sys.argv[1]) == 'save':
    writervideo = animation.FFMpegWriter(fps=10)
    anim.save('anim.mp4', writer=writervideo)
    plt.close()
else:
    plt.show()