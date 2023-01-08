#!/usr/bin/env python
from niryo_one_python_api.niryo_one_api import *
import rospy
import time
import numpy as np
from math import atan2, asin, acos

rospy.init_node('niryo_one_example_python_api')

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
		:type pose: np.array
		:return: joint angles that place the hand of the robot at the required position
		:rtype: np.array
	"""
	joints = []
	for j in inverseKinematicsAux(*pose):
		for s in inverseKinematicsAux(*pose, first=False, theta6_init=j[5]):
			joints.append(s)

	joints = select_solution(joints, pose[0:3])
	return joints

# Output file with the time, commands and respective arguments
f_command = open('commands_uncert.txt', 'w')

def print_command_vel(start_time, vel):
	""" prints to the output file with the instructions a command to change the velocity of the robotic arm
		:param start_time: time value corresponding to the robot inicialization
		:type start_time: float
		:param vel: percentage of the maximum velocity of the robotic arm
		:type vel: int
	"""
	time_string = "{} ".format(time.time()-start_time)
	f_command.write(time_string)
	f_command.write("set_arm_max_velocity ")
	f_command.write("{}\n".format(vel))

def print_command_joints(start_time, joints):
	""" prints to the output file with the instructions a command to change the pose of the robotic arm
		:param start_time: time value corresponding to the robot inicialization
		:type start_time: float
		:param pose: desired position (x,y,z) and orientation (alpha,beta,gamma) sent to the move_pose function
		:type pose: np.array
	"""
	time_string = "{} ".format(time.time()-start_time)
	f_command.write(time_string)
	f_command.write("move_joints ")
	f_command.write("{} {} {} {} {} {}\n".format(joints[0], joints[1], joints[2], joints[3], joints[4], joints[5]))

def move_pose_rand(pose, start_time, prints=True):
	""" moves the robotic arm to a position that if affected by gaussian noise
		:param pose: desired position (x,y,z) and orientation (alpha,beta,gamma)
		:type pose: np.array
		:param start_time: time value corresponding to the robot initialization
		:type start_time: float
		:param prints: defines where to print or not the outputs to the screen
		:type prints: bool
	"""
	joints = inverseKinematics(pose)
	if prints:
		print("Without Noise:")
		print("Joints:")
		print(joints)
		print("Original Pose:")
		print(pose)
		print("Calculated Pose:")
		print(directKinematics(joints))
		print("Position Error:")
		print(np.round(pose[0:3] - directKinematics(joints)[0:3],5))
	move_joints_rand(joints, start_time, prints=True, pose=pose)

def move_joints_rand(joints, start_time, prints=False, pose=None):
	""" moves the joints of the robotic arm after adding gaussian noise to the joints
		:param joints: desired joint values
		:type joints: np.array
		:param start_time: time value corresponding to the robot initialization
		:type start_time: float
		:param prints: defines where to print or not the outputs to the screen
		:type prints: bool
		:param pose: desired position (x,y,z) and orientation (alpha,beta,gamma)
		:type pose: np.array
	"""
	# Apply gaussian noise with standard deviation 0.01
	for i in range(6):
		joints[i] = np.round(np.random.normal(joints[i], 0.01),4)
	joints = bound_joints(joints)
	print_command_joints(start_time, joints)
	n.move_joints(joints)
	if prints:
		print("With Noise:")
		print("Joints:")
		print(joints)
		print("Original Pose:")
		print(pose)
		print("Calculated Pose:")
		print(directKinematics(joints))
		print("Position Error:")
		print(np.round(pose[0:3] - directKinematics(joints)[0:3],5))

def get_trajectory():
	""" imports the sequence of desired poses from a file
		:return: poses
		:rtype: np.array
	"""
	with open("positions.txt") as f:
		lines = f.read().splitlines()
	N_points = int(len(lines))
	poses = np.zeros((N_points,3))
	for i in range(N_points):
		temp = np.fromstring(lines[i], dtype=float, sep=' ')
		poses[i][0] = temp[0]
		poses[i][1] = temp[1]
		poses[i][2] = temp[2]
	# This file also has the home configuration and two configurations where the joints are all zero (needed for the matplotlib simulation). Those are not needed here.
	poses = np.delete(poses, 0, 0)
	poses = np.delete(poses, 0, 0)
	poses = np.delete(poses, -1, 0)
	return poses

coords = get_trajectory()

n = NiryoOne()

print("--- Start")

start_t = time.time()

try:

	print_command_vel(start_t, 100)
	n.set_arm_max_velocity(100)
	print("--- Set all joints to zero")
	move_joints_rand([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], start_t)

	print("--- Move to origin")
	move_pose_rand([coords[0][0], coords[0][1], coords[0][2], 0, 0, np.pi], start_t)

	print("--- Pick block 1")
	print_command_vel(start_t, 5)
	n.set_arm_max_velocity(5)
	move_pose_rand([coords[1][0], coords[1][1], coords[1][2], 0, 0, np.pi], start_t)

	print("--- Move up")
	move_pose_rand([coords[2][0], coords[2][1], coords[2][2], 0, 0, np.pi], start_t)

	print("--- Move to target")
	print_command_vel(start_t, 100)
	n.set_arm_max_velocity(100)
	move_pose_rand([coords[3][0], coords[3][1], coords[3][2], 0, 0, np.pi], start_t)

	print("--- Place block 1")
	print_command_vel(start_t, 5)
	n.set_arm_max_velocity(5)
	move_pose_rand([coords[4][0], coords[4][1], coords[4][2], 0, 0, np.pi], start_t)

	print("--- Move up")
	move_pose_rand([coords[5][0], coords[5][1], coords[5][2], 0, 0, np.pi], start_t)

	print("--- Move to origin")
	print_command_vel(start_t, 100)
	n.set_arm_max_velocity(100)
	move_pose_rand([coords[6][0], coords[6][1], coords[6][2], np.pi/2, 0, np.pi], start_t)

	print("--- Pick block 2")
	print_command_vel(start_t, 5)
	n.set_arm_max_velocity(5)
	move_pose_rand([coords[7][0], coords[7][1], coords[7][2], np.pi/2, 0, np.pi], start_t)

	print("--- Move up")
	move_pose_rand([coords[8][0], coords[8][1], coords[8][2], np.pi/2, 0, np.pi], start_t)

	print("--- Move to target")
	print_command_vel(start_t, 100)
	n.set_arm_max_velocity(100)
	move_pose_rand([coords[9][0], coords[9][1], coords[9][2], np.pi/2, 0, np.pi], start_t)

	print("--- Place block 2")
	print_command_vel(start_t, 5)
	n.set_arm_max_velocity(5)
	move_pose_rand([coords[10][0], coords[10][1], coords[10][2], np.pi/2, 0, np.pi], start_t)

	print("--- Move up")
	move_pose_rand([coords[11][0], coords[11][1], coords[11][2], np.pi/2, 0, np.pi], start_t)

	print("--- Move to origin")
	print_command_vel(start_t, 100)
	n.set_arm_max_velocity(100)
	move_pose_rand([coords[12][0], coords[12][1], coords[12][2], 0, 0, np.pi], start_t)

	print("--- Pick block 3")
	print_command_vel(start_t, 5)
	n.set_arm_max_velocity(5)
	move_pose_rand([coords[13][0], coords[13][1], coords[13][2], 0, 0, np.pi], start_t)

	print("--- Move up")
	move_pose_rand([coords[14][0], coords[14][1], coords[14][2], 0, 0, np.pi], start_t)

	print("--- Move to target")
	print_command_vel(start_t, 100)
	n.set_arm_max_velocity(100)
	move_pose_rand([coords[15][0], coords[15][1], coords[15][2], 0, 0, np.pi], start_t)

	print("--- Place block 3")
	print_command_vel(start_t, 5)
	n.set_arm_max_velocity(5)
	move_pose_rand([coords[16][0], coords[16][1], coords[16][2], 0, 0, np.pi], start_t)

	print("--- Move up")
	move_pose_rand([coords[17][0], coords[17][1], coords[17][2], 0, 0, np.pi], start_t)

	print("--- Move to origin")
	print_command_vel(start_t, 100)
	n.set_arm_max_velocity(100)
	move_pose_rand([coords[18][0], coords[18][1], coords[18][2], np.pi/2, 0, np.pi], start_t)

	print("--- Pick block 4")
	print_command_vel(start_t, 5)
	n.set_arm_max_velocity(5)
	move_pose_rand([coords[19][0], coords[19][1], coords[19][2], np.pi/2, 0, np.pi], start_t)

	print("--- Move up")
	move_pose_rand([coords[20][0], coords[20][1], coords[20][2], np.pi/2, 0, np.pi], start_t)

	print("--- Move to target")
	print_command_vel(start_t, 100)
	n.set_arm_max_velocity(100)
	move_pose_rand([coords[21][0], coords[21][1], coords[21][2], np.pi/2, 0, np.pi], start_t)

	print("--- Place block 4")
	print_command_vel(start_t, 5)
	n.set_arm_max_velocity(5)
	move_pose_rand([coords[22][0], coords[22][1], coords[22][2], np.pi/2, 0, np.pi], start_t)

	print("--- Move up")
	move_pose_rand([coords[23][0], coords[23][1], coords[23][2], np.pi/2, 0, np.pi], start_t)

	print_command_vel(start_t, 100)
	n.set_arm_max_velocity(100)
	print("--- Set all joints to zero")
	move_joints_rand([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], start_t)

except NiryoOneException as e:
	print(e)

f_command.close()
