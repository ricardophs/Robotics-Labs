#!/usr/bin/env python
from niryo_one_python_api.niryo_one_api import *
import math
import rospy
import time
import numpy as np

rospy.init_node('niryo_one_example_python_api')

with open("output.txt") as f:
    lines = f.read().splitlines()

N_points = int(len(lines)/12)

joints = np.zeros((N_points,6))

for i in range(N_points):
    joints[i] = np.fromstring(lines[12*i+7], dtype=float, sep=' ')

n = NiryoOne()
n.calibrate_auto()

print("--- Start")

try:
    n.set_arm_max_velocity(100)
    print("--- Moving robot to home configuration")
    n.move_joints([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    print("--- Move to origin")
    n.move_joints(joints[0])

    print("--- Pick block 1")
    n.set_arm_max_velocity(5)
    n.move_joints(joints[1])

    print("--- Move up")
    n.move_joints(joints[2])

    print("--- Move to target")
    n.set_arm_max_velocity(100)
    n.move_joints(joints[3])

    print("--- Place block 1")
    n.set_arm_max_velocity(5)
    n.move_joints(joints[4])

    print("--- Move up")
    n.move_joints(joints[5])

    print("--- Move to origin")
    n.set_arm_max_velocity(100)
    n.move_joints(joints[6])

    print("--- Pick block 2")
    n.set_arm_max_velocity(5)
    n.move_joints(joints[7])

    print("--- Move up")
    n.move_joints(joints[8])

    print("--- Move to target")
    n.set_arm_max_velocity(100)
    n.move_joints(joints[9])

    print("--- Place block 2")
    n.set_arm_max_velocity(5)
    n.move_joints(joints[10])

    print("--- Move up")
    n.move_joints(joints[11])

    print("--- Move to origin")
    n.set_arm_max_velocity(100)
    n.move_joints(joints[12])

    print("--- Pick block 3")
    n.set_arm_max_velocity(5)
    n.move_joints(joints[13])

    print("--- Move up")
    n.move_joints(joints[14])

    print("--- Move to target")
    n.set_arm_max_velocity(100)
    n.move_joints(joints[15])

    print("--- Place block 3")
    n.set_arm_max_velocity(5)
    n.move_joints(joints[16])

    print("--- Move up")
    n.move_joints(joints[17])

    print("--- Move to origin")
    n.set_arm_max_velocity(100)
    n.move_joints(joints[18])

    print("--- Pick block 4")
    n.set_arm_max_velocity(5)
    n.move_joints(joints[19])

    print("--- Move up")
    n.move_joints(joints[20])

    print("--- Move to target")
    n.set_arm_max_velocity(100)
    n.move_joints(joints[21])

    print("--- Place block 4")
    n.set_arm_max_velocity(5)
    n.move_joints(joints[22])

    print("--- Move up")
    n.move_joints(joints[23])

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
# rosrun niryo_one_python_api <file_name>.py                                                                                                                                                                          # rosrun niryo_one_python_api <file_name>.py
