#!/usr/bin/env python3

# To enable communication on the t7 shield run:
#sudo usermod -a -G dialout $USER 
#sudo chmod a+rw /dev/ttyUSB0

### ================================ ###
# INITIALISATION

# Importing all the ROS libraries
import rospy

# Importing all the flame libraries
from x_flame import ur_five
from x_flame import baslers
from x_flame import motion_estimator

import time

### ================================ ###
# SYSTEM FUNCTIONS

# Starting up and connecting to all the devices
def start_devices():
    global cameras, robot_arm, predictor
    cameras.turn_on()
    robot_arm.turn_on()
    predictor.turn_on()


# The function that initialises the shutdown
def close_devices():
    global cameras, robot_arm, predictor
    cameras.turn_off()
    robot_arm.turn_off()
    predictor.turn_off()
    

### ================================ ###
# THE MAIN WORKFLOW OF THE SYSTEM
if __name__ == '__main__':
    cameras = baslers()
    robot_arm = ur_five(False)
    predictor = motion_estimator()

    rospy.init_node('listener', anonymous = True)
    rospy.on_shutdown(close_devices)
    start_devices()

    rospy.spin()