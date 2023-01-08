# Robotics-Labs
## Lab1 - Introduction to Serial Manipulators
    To Do
## Lab2 - Autonomous Cars
This Laboratory work was developed by two teams. Team A:  Inês Ferreira (90395), Miguel Graça(90142), Ricardo Santos (90178) and team B: Eduardo Cunha (90060), Mónica Gomez (90153), Pedro Antunes (90170).

### Code Structure
The code relevant to this Project can be found at [./Lab\ 2/Code](https://github.com/inesalfe/Robotics-Labs/tree/main/Lab%202/Code). This section will give a brief overview of the project files.

* AuxFunctions \
    In this folder some auxiliary functions and a class are defined. The file path_planning.m represents the principal function that is used at the start of the program to generate the path that the car should follow (trajectory). The file getObstacle.m is used in the control part of the code after the LIDAR sensors detect an object.
* DataFiles \
    This folder contains the data that is used during the program. For example, the trajectory matrix that contains 3 columns, the first corresponding to the x position, the second to the y position and the third to the theta angle - orientation - between the trajectory and the car.
* Images \
    The Images folder contains figures that represent the roads present on the map that is used as an example. In this case the IST - Alameda campus.
* PreProcessing
* **main.m** - This file is the program that should be run by the user. It will create the path that should be followed, execute the control of the car and display its behaviour on the screen. The main file makes use of functions in other files.

Some important toolboxes that were used are:

1. Navigation_Toolbox
2. Curve_Fitting_Toolbox
3. Statistics_Toolbox
4. Symbolic_Toolbox