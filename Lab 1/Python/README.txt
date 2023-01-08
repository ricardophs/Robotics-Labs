SUMMARY FOR EVALLUATION PURPOSES:

Python script that uses the Niryo One Simulator (with gaussian noise) - Niryo/uncert.py;

Text file with instructions for the robot trajectory (with gaussian noise) - Niryo/commands_uncert.py. Each line has a robot action with time (in seconds), action and action arguments;

Video simulation (using Python3) of the gripper trajectory and lego pieces motion (with gaussian noise) - PythonSim/WithUncert/anim.mp4;

Video simulation (using Python3) of the gripper trajectory and lego pieces motion (without gaussian noise) - PythonSim/NoUncert/anim.mp4.

DETAILED EXPLANATION OF FILE PURPOSES AND USAGE:

This first lab is organized into two folders: Niryo and PythonSim.

Niryo contains the python script to be executed in the Niryo One Simulator, using the Python API:

- File positions.txt: has the coordinates and orientations of the poses that make up the robots trajectory. These were calculated using the provided lab layout;

- File uncert.py: this Python script uses the Niryo One api and simulated the robot trajectory with gaussian noise. It outputs a file named commands_uncert.txt with the times, commands and arguments for all robot actions.

PythonSim contains two Python3 simulations of the moviment of the gripper and the lego pieces. The simulation with gaussian noise is in folder WithUncert and the simulation without gaussian  noise is in folder NoUncert. In each folder the files are the same and are the following:

- File positions.txt: the same as the one in folder Niryo - has the coordinates and orientations of the poes that make up the robots trajectory. These were calculated using the provided lab layout;

- File trajectory_saver.py: this Python script uses the positions.txt file to calculate the trajectory between each position. In creates several auxiliary files that are used by the simulation: traj.txt (used to draw the gripper position), ori.txt (used to draw the gripper orientation), indices.txt(used to draw the lego pieces motion);

- File sim.py: creates a 3D simulation of the motion of the gripper and the lego pieces. The usage is the following: if we want to observe the simulation in real time, just run the file with 'python3 sim.py'. If we don't want to see the simulation but only save it in a mp4 file called anim.mp4, run the script with 'python3 sim.py save'.
