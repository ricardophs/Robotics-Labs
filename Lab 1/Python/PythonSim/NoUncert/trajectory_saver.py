import numpy as np

def get_arrow_pos(x, y, z, ori):
	""" calculates the coordinates of tips of the axis of the gripper
		:param x: x coodinate of the center of the gripper
		:rtype: float
		:param y: y coodinate of the center of the gripper
		:rtype: float
		:param z: z coodinate of the center of the gripper
		:rtype: float
		:param ori: orientation of the gripper
		:rtype: int
		:return: coordinates of tips of the axis of the gripper (in order, x, y and z axis)
		:rtype: np.array
	"""
	l = 0.04
	if int(ori) == 0:
		return np.array([[x+l, y, z],
						 [x, y-l, z],
						 [x, y, z-l]])
	elif int(ori) == 1:
		return np.array([[x, y+l, z],
						 [x+l, y, z],
						 [x, y, z-l]])
	elif int(ori) == 2:
		return np.array([[x, y+l, z],
						 [x+l*np.cos(0.757), y, z+l*np.sin(0.757)],
						 [x+l*np.sin(0.757), y, z-l*np.cos(0.757)]])
	elif int(ori) == 3:
		return np.array([[x, y+l, z],
						 [x, y, z+l],
						 [x+l, y, z]])
	else:
		return None

# Get the poses from the file
with open("positions.txt") as f:
	lines = f.read().splitlines()

# Total number of poses
N_points = int(len(lines))

dt = 20 # milliseconds

# Diference of distance between two consecutive poses
delta_d = np.zeros(3)
# Diference of orientation between two consecutive poses
delta_ori = np.zeros((3,3))
# Auxiliary coodinates to save the trajectory
point_int = np.zeros(3)
# Auxiliary orientation to save the trajectory
ori_int = np.zeros((3,3))

# Velocities of the robotic arm
v = [0.00002, 0.00035]

# List of all coordinates in the trajectory
traj = []
# List of all orientations in the trajectory
ori = []
# Auxiliary variables to modulate the moviment of the pieces
s = 0
ind = []

for i in range(N_points-1):
	# Current point
	point_i = np.fromstring(lines[i], dtype=float, sep=' ')
	# Next point
	point_f = np.fromstring(lines[i+1], dtype=float, sep=' ')
	# Current orientation
	ori_i = get_arrow_pos(point_i[0], point_i[1], point_i[2], point_i[3])
	# Current orientation
	ori_f = get_arrow_pos(point_f[0], point_f[1], point_f[2], point_f[3])
	# Diference between the next and current pose
	for j in range(3):
		delta_d[j] = point_f[j] - point_i[j]
	# Diference between the next and current orientations
	delta_ori = ori_f - ori_i
	# Calculate the number of intermediate steps between each pair of poses
	steps = max(abs(delta_d)) / dt / v[int(point_i[4])]
	point_int = point_i
	ori_int = ori_i
	ind.append(s)
	steps = int(steps)
	s += steps
	# Append each intermediate pose and orientation
	for k in range(steps):
		for j in range(3):
			point_int[j] += delta_d[j]/steps
		ori_int += delta_ori/steps
		ori.append(ori_int.copy())
		traj.append(point_int.copy())

# Save all the coordinates to an output file
with open('traj.txt', 'w') as f:
	for point in traj:
		line = "{} {} {}\n".format(point[0], point[1], point[2])
		f.write(line)

# Save all the orientation to an output file
with open('ori.txt', 'w') as f:
	for o in ori:
		line = "{} {} {} {} {} {} {} {} {}\n".format(o[0, 0], o[0, 1], o[0, 2], o[1, 0], o[1, 1], o[1, 2], o[2, 0], o[2, 1], o[2, 2])
		f.write(line)

# Save all the indices of the poses in the total trajectory where the gripper pick up or lets go of a piece
with open('indices.txt', 'w') as f:
	for i in ind:
		line = "{}\n".format(i)
		f.write(line)
