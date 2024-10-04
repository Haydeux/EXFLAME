#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64

# Dictionary to store received messages
time_data = {}

# Callback function for each subscriber
def time_callback(msg, node_name):
    time_data[node_name] = msg.data
    if len(time_data) == 12:  # Adjust according to the number of nodes broadcasting
        print_formatted_time_data()

def print_formatted_time_data():
    print("Received all time broadcasts:")
    for node, time in time_data.items():
        print(f"{node}: {time:.4f} ms")

def listener():
    print("Begin Listening")
    rospy.init_node('time_listener', anonymous=True)

    # Subscribe to the 'average_time' topic from different nodes
    det_mean_sub = rospy.Subscriber('times/detection/mean', Float64, time_callback, callback_args='DetectionMean')
    det_std_sub = rospy.Subscriber('times/detection/std', Float64, time_callback, callback_args='DetectionSTD')

    total_mean_sub = rospy.Subscriber('times/total/mean', Float64, time_callback, callback_args='TotalMean')
    total_std_sub = rospy.Subscriber('times/total/std', Float64, time_callback, callback_args='TotalSTD')

    upload_mean_sub = rospy.Subscriber('times/upload/mean', Float64, time_callback, callback_args='UploadMean')
    upload_std_sub = rospy.Subscriber('times/upload/std', Float64, time_callback, callback_args='UploadSTD')

    conv_mean_sub = rospy.Subscriber('times/conversion/mean', Float64, time_callback, callback_args='ConversionMean')
    conv_std_sub = rospy.Subscriber('times/conversion/std', Float64, time_callback, callback_args='ConversionSTD')

    rect_mean_sub = rospy.Subscriber('times/rectify/mean', Float64, time_callback, callback_args='RectifyMean')
    rect_std_sub = rospy.Subscriber('times/rectify/std', Float64, time_callback, callback_args='RectifySTD')

    stereo_mean_sub = rospy.Subscriber('times/stereo/mean', Float64, time_callback, callback_args='StereoMean')
    stereo_std_sub = rospy.Subscriber('times/stereo/std', Float64, time_callback, callback_args='StereoSTD')
    
    # Add more subscribers as needed, or dynamically based on known nodes

    rospy.spin()

    det_mean_sub.unregister()
    det_std_sub.unregister()

    total_mean_sub.unregister()
    total_std_sub.unregister()

    upload_mean_sub.unregister()
    upload_std_sub.unregister()

    conv_mean_sub.unregister()
    conv_std_sub.unregister()

    rect_mean_sub.unregister()
    rect_std_sub.unregister()

    stereo_mean_sub.unregister()
    stereo_std_sub.unregister()

    print("Closing")

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
