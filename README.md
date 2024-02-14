# Overview
The EXFLAME project is a research project at the University of Waikato. The aim of the project is to automate the picking and pollination of kiwifruit. This will apply to many other fruits too. The aim is to incorporate real time feedback into the motion, in order to allow for disturbances such as wind or other pickig that causes the canopy to shake.

This project is adjusted and continued on from the FLAME project, as the overall goal has changed slightly. The existing [FLAME](https://github.com/dnlbsmn/FLAME.git) code is added here and kept in the FLAME directory for archiving as a backup. 


# EXFLAME
See the [EXFLAME/README.md](EXFLAME/README.md) for details on the programs, the prerequisites, and usage instructions.  

Currently, EXFLAME mainly consists of code for running a pair of basler cameras and code for the control box. This is primarily the work of Hayden Moxsom over the period of the 2023-2024 C trimeseter.  

This work mainly involved reasearching, testing, and implementation of a fast stereo matching algortihm for a new pair of cameras, the design and build of a control box for the canopy, and the programming of the control box logic and communication (required learning ethernet communication).  


# FLAME
This is a copy of the [FLAME](https://github.com/dnlbsmn/FLAME.git) project available at [https://github.com/dnlbsmn/FLAME.git](https://github.com/dnlbsmn/FLAME.git). This was the work of Daniel Bosman and Jackson Whitwell during the 2023 capstone project, as well as the work of Jackson Whitwell over the period of 2023-2024 C trimester.    

It currently contains the logic for motion prediction and tracking and following the kiwifruit with the arm. The relevant code for this is in the [FLAME ROS Package](FLAME/FLAME%20ROS%20Package). It also contains an older version of the canopy control code, from when serial communication was still being used.  

No changes have been made to this section, and it is kept for archiving and backup. If any changes are made in the future, the modified sections will be copied into the EXFLAME directory, and the older version kept in this FLAME directory.
