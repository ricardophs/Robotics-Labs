#!/usr/bin/env python
from niryo_one_python_api.niryo_one_api import *
import math
import rospy
import time
import numpy as np

rospy.init_node('niryo_one_example_python_api')

with open("positions.txt") as f:
    lines = f.read().splitlines()
N_points = int(len(lines))
coords = np.zeros((N_points,3))
for i in range(N_points):
    temp = np.fromstring(lines[i], dtype=float, sep=' ')
    coords[i][0] = temp[0]
    coords[i][1] = temp[1]
    coords[i][2] = temp[2]

coords = np.delete(coords, 0, 0)
coords = np.delete(coords, 0, 0)
coords = np.delete(coords, -1, 0)

n = NiryoOne()
n.calibrate_auto()

print("--- Start")

try:

    n.set_arm_max_velocity(100)
    print("--- Moving robot to home configuration")
    n.move_joints([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    print("--- Move to origin")
    n.move_pose(coords[0][0], coords[0][1], coords[0][2], 0, 1.57, -1.57)

    print("--- Pick block 1")
    n.set_arm_max_velocity(5)
    n.move_pose(coords[1][0], coords[1][1], coords[1][2], 0, 1.57, -1.57)

    print("--- Move up")
    n.move_pose(coords[2][0], coords[2][1], coords[2][2], 0, 1.57, -1.57)

    print("--- Move to target")
    n.set_arm_max_velocity(100)
    n.move_pose(coords[3][0], coords[3][1], coords[3][2], 0, 1.57, -1.57)

    print("--- Place block 1")
    n.set_arm_max_velocity(5)
    n.move_pose(coords[4][0], coords[4][1], coords[4][2], 0, 1.57, -1.57)

    print("--- Move up")
    n.move_pose(coords[5][0], coords[5][1], coords[5][2], 0, 1.57, -1.57)

    print("--- Move to origin")
    n.set_arm_max_velocity(100)
    n.move_pose(coords[6][0], coords[6][1], coords[6][2], 0, 1.57, 0)

    print("--- Pick block 2")
    n.set_arm_max_velocity(5)
    n.move_pose(coords[7][0], coords[7][1], coords[7][2], 0, 1.57, 0)

    print("--- Move up")
    n.move_pose(coords[8][0], coords[8][1], coords[8][2], 0, 1.57, 0)

    print("--- Move to target")
    n.set_arm_max_velocity(100)
    n.move_pose(coords[9][0], coords[9][1], coords[9][2], 0, 1.57, 0)

    print("--- Place block 2")
    n.set_arm_max_velocity(5)
    n.move_pose(coords[10][0], coords[10][1], coords[10][2], 0, 1.57, 0)

    print("--- Move up")
    n.move_pose(coords[11][0], coords[11][1], coords[11][2], 0, 1.57, 0)

    print("--- Move to origin")
    n.set_arm_max_velocity(100)
    n.move_pose(coords[12][0], coords[12][1], coords[12][2], 0, 1.57, -1.57)

    print("--- Pick block 3")
    n.set_arm_max_velocity(5)
    n.move_pose(coords[13][0], coords[13][1], coords[13][2], 0, 1.57, -1.57)

    print("--- Move up")
    n.move_pose(coords[14][0], coords[14][1], coords[14][2], 0, 1.57, -1.57)

    print("--- Move to target")
    n.set_arm_max_velocity(100)
    n.move_pose(coords[15][0], coords[15][1], coords[15][2], 0, 1.57, -1.57)

    print("--- Place block 3")
    n.set_arm_max_velocity(5)
    n.move_pose(coords[16][0], coords[16][1], coords[16][2], 0, 1.57, -1.57)

    print("--- Move up")
    n.move_pose(coords[17][0], coords[17][1], coords[17][2], 0, 1.57, -1.57)

    print("--- Move to origin")
    n.set_arm_max_velocity(100)
    n.move_pose(coords[18][0], coords[18][1], coords[18][2], 0, 1.57, 0)

    print("--- Pick block 4")
    n.set_arm_max_velocity(5)
    n.move_pose(coords[19][0], coords[19][1], coords[19][2], 0, 1.57, 0)

    print("--- Move up")
    n.move_pose(coords[20][0], coords[20][1], coords[20][2], 0, 1.57, 0)

    print("--- Move to target")
    n.set_arm_max_velocity(100)
    n.move_pose(coords[21][0], coords[21][1], coords[21][2], 0, 1.57, 0)

    print("--- Place block 4")
    n.set_arm_max_velocity(5)
    n.move_pose(coords[22][0], coords[22][1], coords[22][2], 0, 1.57, 0)

    print("--- Move up")
    n.move_pose(coords[23][0], coords[23][1], coords[23][2], 0, 1.57, 0)

    n.set_arm_max_velocity(100)
    print("--- Moving robot to home configuration")
    n.move_joints([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

except NiryoOneException as e:
    print(e)

# PATH TO FILES
# /home/ines/catkin_ws/src/niryo_one_python_api/examples/NiryoPy/scripts
# COMMAND FOR RVIZ - W/ JOINTS CONTROLLER
# roslaunch niryo_one_description display.launch
# COMMAND FOR RVIZ - PYTHON
# roslaunch niryo_one_bringup desktop_rviz_simulation.launch
# COMMAND TO RUN PYTHON FILE
# rosrun niryo_one_python_api <file_name>.py                                                                                                                                                                         # rosrun niryo_one_python_api <file_name>.py
