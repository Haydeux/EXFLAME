# EXFLAME 
The two main programs of significance are the [`baslers_class`](baslers_class/) and the [`Ethernet_Communication`](Canopy_Control/Ethernet_Communication/) programs. 


# baslers_class
This program uses two basler cameras to perform kiwi fruit detection and stereo matching. The kiwifruits position (in metres) is then broadcasted by a ROS topic called `/FLAME/KiwiPos`. 

To see the prerequisites and detailed instructions on how to use, refer to the [`baslers_class/README.md`](baslers_class/README.md).


# Canopy Control - Ethernet Communication
This includes two programs.  
 - [`Ethernet_canopy_control_box.ino`](Canopy_Control/Ethernet_Communication/Ethernet_canopy_control_box/), which can be flashed onto the Mduino within the control box (should be flashed to it already).   
 - [`Ethernet_canopy_control_GUI.py`](Canopy_Control/Ethernet_Communication/Ethernet_canopy_control_GUI.py), which is a GUI program that can be run to connect to the Mduino by ethernet and control the canopy.  

To see the prerequisites and detailed instructions on how to use for both programs, refer to the [`Ethernet_Communication/README.md`](Canopy_Control/Ethernet_Communication/README.md).