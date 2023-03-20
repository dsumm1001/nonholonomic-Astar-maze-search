# Project #3 Phase 1: A* Algo Maze Search with Non-Holonomic Constraints
Author: Doug Summerlin (dsumm1001@gmail.com, dsummerl@umd.edu)  
UID: 114760753  
Directory ID: dsummerl  
Author: Vignesh Rajagopal (vickyrv570@gmail.com, vigneshr@umd.edu)  
UID: 119476192  
Directory ID: vigneshr  
ENPM661 Spring 2023: Robotic Path Planning

Github Repo Link: https://github.com/dsumm1001/nonholonomic-Astar-maze-search.git  
Results Link: https://docs.google.com/document/d/1odwbrP457jTVn1dUarhkSgPyvDlK8KBRE_ITPUqQi6A/edit?usp=sharing

This project is basic visual implementation of the A* algorithm for a point robot navigating a maze under non-holonomic constraints

FILESYSTEMS: cd into the folder titled "proj3\_douglas\_vignesh" after downloading it on an Ubuntu or Windows operating system. 
Nothing else should be required here, the code relies on no external files. 

Ensure the following libraries are installed so they can be imported as such:
    import numpy as np
    import matplotlib.pyplot as plt
    import cv2
    import math
    from queue import PriorityQueue
    import time
    import sys

OPERATING SYSTEM: Ubuntu 20.04 Linux (but Windows 11 should also work)

TO RUN SCRIPTS:   
\$ cd ~/home/.../Proj3\_douglas\_vignesh/src  
\$ python3 a\_star\_douglas\_vignesh.py

The program will begin by prompting the user for the size of the robot radius (AKA boundary clearance) as well as the step size for a single actuation of the robot. Then the program will prompt the user start and goal node coordinates in x,y format, as well as the orientation of the robot at the start position and the orientation of the robot at the end position. Please enter integer numbers between that are within the mazespace and not within the boundary or obstacle space for x and y coordinates, separated only be a comma and no whitespace, eg. "595,245" for a boundary width of 5 is acceptable. Please enter the orientation as an integer angle value between 0-360 degrees within increments of 30 deg. The program will repeat the prompt for the position and orientation if acceptable values are not provided. 

The program then will use the A* algorithm to find the optimal path from the established start node to the established goal node using the non-holonomic action set dictated. Once the optimal path has been found, the program will prompt:

"Yay! Goal node located... Operation took  X.X seconds."

Typical pathing time is between 1 and 5 seconds depending on the machine. 

The program will then display the path on an image window imposed on the maze, with all of the searched nodes appearing as blue arrows. Close this window to continue

The program will then prepare a visual simulation of the search process depicting searched areas with blue arrows, generate the optimal pathline as a series of yellow circles, and then simulate a (purple) circular mobile robot following the optimal path. This simulation will be generated and saved as an .mp4 file, before finally appearing as a video window on the OS GUI. Once the video is closed or the video ends, the program will terminate. 