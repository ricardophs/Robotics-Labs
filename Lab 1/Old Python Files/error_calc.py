import numpy as np

# Reads the output file from the test kinematics
with open("output.txt") as f:
	lines = f.read().splitlines()

N_points = int(len(lines)/12)

# Coordinate error calculated with the estimated coodinates from the direct kinematics
coord_error = np.zeros(3)
# Coordinate error calculated with the estimated coodinates from the direct kinematics but using as input the joint estimated with the inverse kinematics
coord_error_inv_joints = np.zeros(3)
# Joint error calculated with the estimated joints from the inverse kinematics
joints_error = np.zeros(6)
# Joint error calculated with the estimated joints from the inverse kinematics but using as input the coordinates estimated with the direct kinematics
joints_error_dir_coord = np.zeros(6)

for i in range(N_points):
	# "Correct" Joints
	arr1 = np.fromstring(lines[12*i+7], dtype=float, sep=' ')
	# Estimated Joints
	arr2 = np.fromstring(lines[12*i+9], dtype=float, sep=' ')
	arr3 = np.fromstring(lines[12*i+11], dtype=float, sep=' ')
	for j in range(6):
		joints_error[j] += (arr1[j]-arr2[j])**2
		joints_error_dir_coord[j] += (arr1[j]-arr3[j])**2
	# "Correct" Coordinates
	arr1 = np.fromstring(lines[12*i+1], dtype=float, sep=' ')
	# Estimated Coordinates
	arr2 = np.fromstring(lines[12*i+3], dtype=float, sep=' ')
	arr3 = np.fromstring(lines[12*i+5], dtype=float, sep=' ')
	for k in range(3):
		coord_error[k] += (arr1[k]-arr2[k])**2
		coord_error_inv_joints[k] += (arr1[k]-arr3[k])**2

# RMSE calculation
for j in range(6):
	joints_error[j] = np.sqrt(joints_error[j]/N_points)
	joints_error_dir_coord[j] = np.sqrt(joints_error_dir_coord[j]/N_points)
for k in range(3):
	coord_error[k] = np.sqrt(coord_error[k]/N_points)
	coord_error_inv_joints[k] = np.sqrt(coord_error_inv_joints[k]/N_points)

print("RMSE - Coordenates w/ Direct Kinematics:")
print(coord_error)
print("RMSE - Coordenates w/ Direct Kinematics using joints from Inverse Kinematics:")
print(coord_error_inv_joints)
print("RMSE - Joints w/ Inverse Kinematics:")
print(joints_error)
print("RMSE - Joints w/ Inverse Kinematics using pose from Direct Kinematics:")
print(joints_error_dir_coord)
