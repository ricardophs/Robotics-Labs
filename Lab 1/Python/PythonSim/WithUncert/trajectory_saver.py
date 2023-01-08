import numpy as np
from math import atan2, asin, acos

# Robot Dimensions in meters
L1 = 103/1e3
L2 = 80/1e3
L3 = 210/1e3
L4 = 30/1e3
L5 = 41.5/1e3
L6 = 180/1e3
L7 = 23.7/1e3
L8 = 5.5/1e3

# Minimum and maximum values of each joints
JOINT_BOUNDS = ((-3.0543, 3.0543),
				(-1.5707, 0.6405),
				(-1.3962, 1.5707),
				(-3.0543, 3.0543),
				(-1.7453, 1.9198),
				(-2.5743, 2.5743))

def get_orientation(r):
	""" gets the orientation from a rotation matrix
		:param r: rotation matrix
		:type r: np.array
		:return: Z-Y-X euler angles representing the orientation, alpha, beta, gamma
		:rtype: np.array
	"""
	beta = atan2(-r[2,0], np.sqrt(r[0,0]**2 + r[1,0]**2))
	cb = np.cos(beta)
	if abs(cb) > 1e-4:
		alpha = atan2(r[1,0]/cb, r[0,0]/cb)
		gamma = atan2(r[2,1]/cb, r[2,2]/cb)
	else:
		alpha = 0
		if beta > 0:
			gamma = atan2(r[0,1], r[1,1])
		else:
			gamma = -atan2(r[0,1], r[1,1])
	return np.array([alpha, beta, gamma])

def transformation_matrix(DH_table_line):
	""" gets the transformation matrix for a given frame
		:param DH_table_line: line from the DH table corresponding to a given frame
		:type DH_table_line: np.array
		:return: tranformation matrix
		:rtype: np.array
	"""
	a = DH_table_line[0]
	alpha = DH_table_line[1]
	d = DH_table_line[2]
	theta = DH_table_line[3]

	c_theta = np.cos(theta)
	s_theta = np.sin(theta)
	c_alpha = np.cos(alpha)
	s_alpha = np.sin(alpha)

	T = np.array([[c_theta, -s_theta, 0, a],
				 [s_theta*c_alpha, c_theta*c_alpha, -s_alpha, -s_alpha*d],
				 [s_theta*s_alpha, c_theta*s_alpha, c_alpha, c_alpha*d],
				 [0, 0, 0, 1]])

	return T

def DH_table(thetas):
	""" gets the DH table given the joint angles
		:param thetas: joint angles
		:type thetas: np.array
		:return: matricial representation of the DH table where each line corresponds to one frame
		:rtype: np.array
	"""
	table = np.array([[0, 0, L1+L2, thetas[0]],
			   [0, np.pi/2, 0, thetas[1]+np.pi/2],
			   [L3, 0, 0, thetas[2]],
			   [L4, np.pi/2, L5+L6, thetas[3]],
			   [0, -np.pi/2, 0, thetas[4]],
			   [-L8, np.pi/2, L7, thetas[5]-np.pi/2]])
	return table

def directKinematics(thetas, mat=False):
	""" gets the total transformation matrix
		:param thetas: joint angles
		:type thetas: np.array
		:return: x, y, and z positions and orientation angles
		:rtype: np.array
	"""
	T1 = transformation_matrix(DH_table(thetas)[0])
	T2 = transformation_matrix(DH_table(thetas)[1])
	T3 = transformation_matrix(DH_table(thetas)[2])
	T4 = transformation_matrix(DH_table(thetas)[3])
	T5 = transformation_matrix(DH_table(thetas)[4])
	T6 = transformation_matrix(DH_table(thetas)[5])
	T56 = np.matmul(T5,T6)
	T46 = np.matmul(T4,T56)
	T36 = np.matmul(T3,T46)
	T26 = np.matmul(T2,T36)
	T16 = np.matmul(T1,T26)

	angles = get_orientation(T16[0:3, 0:3])

	output = np.array([T16[0:3, 3], angles])
	if mat:
		return np.round(T16, 4)
	return np.round(output.flatten(),4)

def bound_joints(joints):
	""" bounds the values of the six robot joints to their physical limits, stored in 
	global variable JOINT_BOUNDS.
		:param joints: joint angles
		:type joints: np.array
		:return: bounded joints
		:rtype: np.array
	"""
	for i, _ in enumerate(joints):
		joints[i] = max(JOINT_BOUNDS[i][0], min(JOINT_BOUNDS[i][1], joints[i]))
	return joints

def select_solution(sols, desired):
	""" given a set of inverse kinematics solutions, selects that which yields a smaller 
	position error between the desired position and the position obtained by applying the 
	direct kinematics to the joint values in a solution.
		:param sols: set with all the computed solutions (usually 4)
		:type joints: np.array
		:param desired: desired position, in format (x, y, z)
		:type desired: np.array
		:return: best solution in the set sols
		:rtype: np.array
	"""
	best, best_diff = None, 1e6
	for sol in sols:
		# Bound each joint to its mechanical limits
		sol = bound_joints(sol)
		# Get the direct kinematics of the bounded joint values
		computed = directKinematics(sol)
		# Compute the error to the desired position
		err = np.linalg.norm(computed[0:3] - desired)
		# Save the best solution
		if err < best_diff:
			best, best_diff = sol, err
	return np.array(best) if best is not None else None

def inverseKinematicsAux(px, py, pz, alpha, beta, gamma, first=True, theta6_init=0):
	""" computes one iteration of the inverse kinematics. In the first iteration (first=True), discards
	the L8=5.5mm distance, computing the 6 DOF angles as if this distance didn't exist (simpler problem).
	Uses a geometric method to compute theta1 to theta3, and an algebraic method to compute the remaining
	joint angles. If first=false, then considers L8 and uses the previously determined theta6 value as an
	initial value.

	:param px, py, pz: desired position, in meters, of the hand of the robot
	:type px, py, pz: float
	:param alpha, beta, gamma: desired orientation, in Z-Y-X euler angles and radians, of the hand of the
	robot
	:type alpha, beta, gamma: float
	:param first: whether this is the first iteration or not
	:type first: bool
	:param theta6_init: initial value for theta6, when first is false
	:type theta6_init: float
	:return: computed joint angles
	:rtype: np.array
	"""
	# theta1 is readily computed
	theta1 = atan2(py, px)
	# rotation matrix associated with the desired robot hand orientation
	angles = np.array([alpha, beta, gamma])
	c = np.cos(angles)
	s = np.sin(angles)
	R06 = np.array([ [c[0]*c[1], c[0]*s[1]*s[2] - s[0]*c[2], c[0]*s[1]*c[2] + s[0]*s[2]],
					 [s[0]*c[1], s[0]*s[1]*s[2] + c[0]*c[2], s[0]*s[1]*c[2] - c[0]*s[2]],
					 [-s[1], c[1]*s[2], c[1]*c[2]] ])
	R06 = np.round(R06, 4)
	# desired robot hand position
	P06 = np.array([px, py, pz])
	# translation from frame 5 to frame 6 depending on if L8 is considered
	if first:
		P56 = np.array([0,0,L7])
	else:
		P56 = np.array([-L8*np.sin(theta6_init),-L8*np.cos(theta6_init),L7])
	# position of the center of the joint5 frame in the world frame (frame 0)
	P05 = P06 - np.matmul(R06,P56) - np.array([0,0,L1+L2])
	L46 = np.sqrt(L4**2 + (L5+L6)**2)
	# compute theta3 geometrically
	theta3 = asin((np.linalg.norm(P05)**2 - L3**2 - L46**2)/(2*L3*L46)) - atan2(L4, L5+L6)
	# auxiliar angles
	beta = atan2(P05[2], np.sqrt(P05[0]**2 + P05[1]**2))
	phi = acos((L3**2 + np.linalg.norm(P05)**2 - L46**2)/(2*L3*np.linalg.norm(P05)))
	# compute theta2 geometrically
	theta2 = beta + phi - np.pi/2
	# At this point, theta1 to theta3 are determined

	# desired transformation matrix (known)
	T06 = np.zeros([4,4])
	T06[0:3,0:3], T06[0:3,3] = R06, P06
	T06[3,0:4] = np.array([0,0,0,1])

	# Transformation matrix from the world frame to the frame of joint3
	c = np.cos([theta1, theta2, theta3])
	s = np.sin([theta1, theta2, theta3])
	T1 = np.array([[c[0], -s[0],  0,   0  ],
				   [s[0],  c[0],  0,   0  ],
				   [   0,     0,  1, L1+L2],
				   [   0,     0,  0,   1  ]])

	T2 = np.array([[-s[1], -c[1],  0,  0],
				   [    0,     0, -1,  0],
				   [ c[1], -s[1],  0,  0],
				   [    0,     0,  0,  1]])

	T3 = np.array([[c[2], -s[2],  0, L3],
				   [s[2],  c[2],  0,  0],
				   [   0,     0,  1,  0],
				   [   0,     0,  0,  1]])
	T03 = np.matmul(T1,np.matmul(T2,T3))
	# T03 matrix inversion
	T03[0:3,0:3], T03[0:3,3] = T03[0:3,0:3].T, - np.matmul(T03[0:3,0:3].T,T03[0:3,3])

	# T36 = (T03)^-1 @ T06
	# T36 is the tranformation matrix from the frame of joint3 to the frame of the robot hand
	# (equivalent to the frame of joint6); depends only on theta 4 to theta 6
	T36 = np.matmul(T03,T06)
	T36 = np.round(T36, 4)

	# print(T36)
	# compute theta 4 to theta 6 algebraically
	theta5 = [acos(-T36[1,2]), -acos(-T36[1,2])] # two solutions for theta5
	# theta 4 and theta 6 depend on theta5, so for each theta 5 value there is a theta 4 and theta 6 pair,
	# which means that there will be 2 solutions
	theta4 = []
	theta6 = []
	for t5 in theta5:
		theta4.append(0)
		theta6.append(atan2(T36[0,0]/np.cos(t5), T36[0,1]/np.cos(t5)))

	theta1 = [theta1, theta1]
	theta2 = [theta2, theta2]
	theta3 = [theta3, theta3]
	sols_tentative = np.stack([theta1, theta2, theta3, theta4, theta5, theta6])

	return np.round(sols_tentative.T,4)

def inverseKinematics(pose):
	""" computes the angles of the robot joints that place the hand of the robot at a given
	position with a given orientation. Uses an iterative approximation
		:param pose: desired position (x,y,z) and orientation (alpha,beta,gamma) of the robot hand
		:type r: np.array
		:return: joint angles that place the hand of the robot at the required position
		:rtype: np.array
	"""
	joints = []
	for j in inverseKinematicsAux(*pose):
		for s in inverseKinematicsAux(*pose, first=False, theta6_init=j[5]):
			joints.append(s)

	joints = select_solution(joints, pose[0:3])
	return joints

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

def get_ori_angles(x, y, z, ori):
	""" calculates the euler angles corresponding to a given orientation using the orientation matrix
		:param x: x coodinate of the center of the gripper
		:rtype: float
		:param y: y coodinate of the center of the gripper
		:rtype: float
		:param z: z coodinate of the center of the gripper
		:rtype: float
		:param ori: orientation of the gripper
		:rtype: int
		:return: euler angles
		:rtype: np.array
	"""
	if int(ori) == 0:
		m = np.array([	[1, 0, 0],
						[0, -1, 0],
						[0, 0, -1]])
	elif int(ori) == 1:
		m = np.array([	[0, 1, 0],
						[1, 0, 0],
						[0, 0, -1]])
	elif int(ori) == 2:
		m1 = np.array([	[1, 0, 0],
						[0, np.cos(-(np.pi/2-0.757)), -np.sin(-(np.pi/2-0.757))],
						[0, np.sin(-(np.pi/2-0.757)), np.cos(-(np.pi/2-0.757))]])
		m2 = np.array([	[0, 1, 0],
						[0, 0, 1],
						[1, 0, 0]])
		m = np.matmul(m1, m2)
	elif int(ori) == 3:
		m = np.array([	[0, 1, 0],
						[0, 0, 1],
						[1, 0, 0]])
	else:
		m = None
	return get_orientation(m)

def add_noise(pose):
	""" returns a noisy version of the original pose
		:param pose: x coodinate of the center of the gripper
		:rtype: np.array
		:return: pose
		:rtype: np.array
	"""
	joints = inverseKinematics(pose)
	for i in range(6):
		joints[i] = np.round(np.random.normal(joints[i], 0.01),4)
	joints = bound_joints(joints)
	pose = directKinematics(joints)
	return pose[0:3]

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
	# Get the current pose with x, y and z coordinates and alpha, beta, gamma angles
	pi = np.array([point_i[0:3], get_ori_angles(point_i[0], point_i[1], point_i[2], point_i[3])]).flatten()
	# Get the next pose with x, y and z coordinates and alpha, beta, gamma angles
	pf = np.array([point_f[0:3], get_ori_angles(point_f[0], point_f[1], point_f[2], point_f[3])]).flatten()
	# Add noise to the points
	pi_uncert = add_noise(pi)
	pf_uncert = add_noise(pf)
	# Diference between the next and current pose
	for j in range(3):
		delta_d[j] = pf_uncert[j] - pi_uncert[j]
	# Diference between the next and current orientations
	delta_ori = ori_f - ori_i
	# Calculate the number of intermediate steps between each pair of poses
	steps = max(abs(delta_d)) / dt / v[int(point_i[4])]
	point_int = pi_uncert
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
