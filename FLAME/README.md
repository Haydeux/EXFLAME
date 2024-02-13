# FLAME
From the existing work on the [FLAME project](https://github.com/dnlbsmn/FLAME.git). This was the work of Daniel Bosman and Jackson Whitwell during the 2023 capstone project, as well as the work of Jackson Whitwell over the period of 2023-2024 C trimester.  

Has now become the EXFLAME project and any new changes will exist in that directory.  


# Canopy Control 
This is used on the control box to control the canopy. The code uses serial communication and encoder polling. It has since been replaced by interrupts and ethernet communication in the EXFLAME directory with the [ethenet communication version](/EXFLAME/Canopy_Control/Ethernet_Communication).


# FLAME ROS Package
This is the important section. It currently has not been ported to the EXFLAME directory. 

This part is the fruit tracking and motion prediction stuff.
