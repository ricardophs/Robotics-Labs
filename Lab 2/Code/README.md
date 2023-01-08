# Lab2 - Autonomous Cars
This Laboratory work was developed by two teams. Team A:  Inês Ferreira (90395), Miguel Graça (90142), Ricardo Santos (90178) and team B: Eduardo Cunha (90060), Mónica Gomez (90153), Pedro Antunes (90170).

## Usage

The program will start after the user runs the **main.m** file. 

The user has two options of simulation:

Selecting mode 0, the user can choose any start and finish position in the map. 
For path selection, the user will have to move their cursor and click on the starting point first, and then the finishing point.
If the user doesn't want to insert a start and finish position, default positions are available.
Stop signs and traffic lights are available in several locations and the user can select as many as it wants, or none.
The program will prompt for the traffic light selection after the path selection, followed by the stop sign selection.
In both, the user can click on each obstacle to activate, and they will later be considered in the simulation.
To confirm each selection, the user has to press 'space'.
Once the user has set up the simulation and confirmed, the program will start calculating the path, which may take a minute, depending on the complexity of the path and objects chosen.
This calculation will end on a time dependent simulation of the car's path.
The space bar should be pressed to move on.
Following the simulation, a summary of the car's trajectory is presented, together with some plots of errors and velocities.
Again, after each plot, the user should press the space bar to continue to the next plot.
Finally, a calculation of all the collisions of the car with the borders of the road is made, and a final message is printed showing if there were any collisions or not.

In mode 1 the maximum velocity is set according to an energy budget.
There are 4 available paths and, for each one, the user can introduce an arbitrary energy budget or, the default value, carefully decided for each path, can be selected.
In this version, Gaussian noise was also added to the car's GPS and orientation sensors.
If the user wants to see the 4 available paths, images starting with the word 'path' are available in 'Images' folder.

## Code Structure
This section will give a brief overview of the project files.

* AuxFunctions
    * dijkstra_ad_matrix.m - Basic dijkstra algorithm that uses a weighted adjacency matrix
    * dijkstra_map_heur.m - Same as the 'dijkstra_map' but uses an heuristic (distance to goal) to speed up the algorithm
    * dijkstra_map.m - Uses an rgb map to find the shortest path between two pixels
    * find_time_f.m
    * find_time_s.m
    * findShortestPath_matrix.m - Uses the function 'dijkstra_ad_matrix' to get an ordered sequence of nodes that make up the shortest path
    * findShortestPath.m - Uses functions 'dijkstra_map' and 'dijkstra_map_heur' to find an ordered sequence of pixels that make up the calculated path between two nodes 
    * get_sem.m - Used to get the users input when choosing the desired traffic lights
    * get_stops.m - Used to get the users input when choosing the desired stop signs
    * getObstacle.m - Used in the control part of the code after the Lidar sensors detect an object.
    * is_black.m
    * is_blue.m
    * is_green.m
    * is_red.m
    * is_white.m
    * is_yellow.m
    * nearestBlackPixel.m - Find the nearest pixel in the reference path
    * nearestStopSemaphore.m - Find the nearest obstacle in a given occupancy matrix to calculate the exact position of a Lidar estimation of a stop's sign or a traffic light
    * path_planning.m - Used at the start of the program to generates the path that the car should follow
    * PriorityQueue.m
* DataFiles\
    This folder contains the data that is used during the program. For example, the path to follow matrix that contains 3 columns, the first correspond to the x position, the second to the y position and the third to the Theta angle between the trajectory and the car.
* Images\
    The Images' folder contains figures that represent the roads present on the map that is used as an example. In this case, the IST - Alameda campus.
* PreProcessing
* **main.m** - This file is the program that should be run by the user. It will create the path that should be followed, execute the control of the car and display its behaviour on the screen. The main file makes use of functions in other files.

Some important toolboxes that were used are:

1. Navigation_Toolbox
2. Curve_Fitting_Toolbox
3. Statistics_Toolbox
4. Symbolic_Toolbox
