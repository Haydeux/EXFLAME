# Develpoment and Testing
This directory contains versions of the python and C++ codes that were used while developing and testing the basler cameras for stereo matching.

The codes have been kept here for backup and archiving.

## cpp_codes
Mostly focuses on running the baslers, calibration, testing stereo matching, testing kiwifruit detection, testing using the gpu, testing errors, and testing ros messaging.

## python_codes
Earlier codes while still learning, before switching to implement in C++. Also has a [3D point cloud sender](python_codes/global_point_cloud.py) for viewing using ros. To see the point cloud, launch rviz using the terminal command:
```bash
rosrun rviz rviz
```
Once open, press the `Add` button, then select `By topic`. From here, find the `/point_cloud` topic (expand it, if its not already) and select the `PointCloud2` type beneath it and click ok.   
Expand the `PointCloud2` type, then change the `Channel Name` 
to `Z` to make visualisation easier. 

## ethernet_trials
Code used to learn and test how ethernet communication worked, before implementing in the [ethernet version of the canopy control code](../Canopy_Control/Ethernet_Communication).

## Camera_Calibration
Early camera calibration results from matlab and other useful parts associated with the calibration.

