#!/usr/bin/env python
import cv2 as cv
import numpy as np
from time import perf_counter
from pypylon import pylon

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header
import tf2_ros
import geometry_msgs.msg
import tf

import ctypes
import struct

def get_rectifier_maps(a=0):
    # Load calibration parameters from the text file
    calibration_data = np.loadtxt('stereoParams.txt', delimiter=',')

    # Extract parameters
    intrinsics_left = calibration_data[:3, :3]
    distortion_left = calibration_data[3, :5]
    intrinsics_right = calibration_data[4:7, :3]
    distortion_right = calibration_data[7, :5]
    extrinsics_R = calibration_data[8:11, :3]
    extrinsics_T = calibration_data[11, :3]

    # Create stereo rectification maps
    R1, R2, P1, P2, Q, roi_left, roi_right = cv.stereoRectify(
        intrinsics_left, distortion_left,
        intrinsics_right, distortion_right,
        (640, 480), # specify the size of your images
        extrinsics_R, extrinsics_T,
        flags=cv.CALIB_ZERO_DISPARITY, alpha=a
    )

    # Compute the rectification maps
    map_left_x, map_left_y = cv.initUndistortRectifyMap(
        intrinsics_left, distortion_left, R1, P1,
        (640, 480), cv.CV_32FC1
    )
    map_right_x, map_right_y = cv.initUndistortRectifyMap(
        intrinsics_right, distortion_right, R2, P2,
        (640, 480), cv.CV_32FC1
    )
    
    return map_left_x, map_left_y, map_right_x, map_right_y, roi_left, roi_right, Q



def main():
    map_left_x, map_left_y, map_right_x, map_right_y, roi_left, roi_right, Qmat = get_rectifier_maps(0)


    # print(map_left_x[240,17])
    # print(map_left_y[15,240])
    # print(map_left_x)
    # print(map_left_y)

    focal_length_pixel = 855
    baseline = 0.03316 # meters
    baseline_mm = baseline*1000 # mm

    broadcaster = tf.TransformBroadcaster()
    rospy.init_node('point_cloud_publisher', anonymous=True)
    pub = rospy.Publisher('/point_cloud', PointCloud2, queue_size=1)


    def nothing(x):
        pass
    cv.namedWindow('disp',cv.WINDOW_NORMAL)
    cv.resizeWindow('disp',600,600)
    cv.createTrackbar('numDisparities','disp',10,15,nothing)
    cv.createTrackbar('blockSize','disp',1,15,nothing)
    cv.createTrackbar('P1','disp',5,25,nothing)
    cv.createTrackbar('P2','disp',50,100,nothing)
    cv.createTrackbar('preFilterCap','disp',0,400,nothing)
    cv.createTrackbar('mode','disp',2,3,nothing)
    cv.createTrackbar('uniquenessRatio','disp',1,15,nothing)
    cv.createTrackbar('speckleRange','disp',4,10,nothing)
    cv.createTrackbar('speckleWindowSize','disp',50,300,nothing)
    cv.createTrackbar('disp12MaxDiff','disp',200,400,nothing)
    cv.createTrackbar('minDisparity','disp',5,25,nothing)
    
    stereo = cv.StereoSGBM_create()
    numDisparities = 10*16 #10*16
    blockSize = 1*2 + 1 # 5*2 +1 or 4*2 +1
    P1 = 5 # 0 * 3 * blockSize**2 # 0, try somewhere in between.
    P2 = 50 * 3 * blockSize**2 # 15, up to 40 work quite well
    preFilterCap = 0
    mode = 2 # 1 looks best but is very slow. 2 has minor horizontal smudges but is very fast.
    uniquenessRatio = 1
    speckleRange = 4
    speckleWindowSize = 50
    disp12MaxDiff = 200
    minDisparity = 5
    
    # Setting the updated parameters before computing disparity map
    stereo.setNumDisparities(numDisparities)
    stereo.setBlockSize(blockSize)
    stereo.setP1(P1)
    stereo.setP2(P2)
    stereo.setPreFilterCap(preFilterCap)
    stereo.setMode(mode)
    stereo.setUniquenessRatio(uniquenessRatio)
    stereo.setSpeckleRange(speckleRange)
    stereo.setSpeckleWindowSize(speckleWindowSize)
    stereo.setDisp12MaxDiff(disp12MaxDiff)
    stereo.setMinDisparity(minDisparity)


    tlf = pylon.TlFactory.GetInstance()

    # Get all available devices
    devices = tlf.EnumerateDevices()

    if len(devices) < 2:
        print("Less than 2 cameras found.")
        return
    elif len(devices) > 2:
        print("More than 2 cameras found.")
        return

    # Note: devices are always ordered with the lowest Serial Number (SN) first.
    # Therefore this will always be correct (so long as the cameras arent physically swapped in the holder).
    camera_left = pylon.InstantCamera(tlf.CreateDevice(devices[0]))
    camera_right = pylon.InstantCamera(tlf.CreateDevice(devices[1]))

    camera_left.Open()
    camera_right.Open()
    
    dothething = True

    try:
        camera_left.ExposureTime.SetValue(4000)
        camera_right.ExposureTime.SetValue(4000)

        # Start grabbing images
        camera_left.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
        camera_right.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)

        total_time = 0
        disparity_time = 0
        loops = 0
        key = 0
        while camera_left.IsGrabbing() and camera_right.IsGrabbing() and key != 27 and (not rospy.is_shutdown()):

            loops += 1
            t1_start = perf_counter()

            grab_result_left = camera_left.RetrieveResult(100)
            if grab_result_left.GrabSucceeded():
                left_image = grab_result_left.Array
            grab_result_left.Release()

            grab_result_right = camera_right.RetrieveResult(100)
            if grab_result_right.GrabSucceeded():
                right_image = grab_result_right.Array
            grab_result_right.Release()

            color_left_image = cv.cvtColor(left_image, cv.COLOR_BAYER_BG2RGB)
            color_right_image = cv.cvtColor(right_image, cv.COLOR_BAYER_BG2RGB)

            rectified_right = cv.remap(color_right_image, map_right_x, map_right_y, cv.INTER_LINEAR)
            rectified_left = cv.remap(color_left_image, map_left_x, map_left_y, cv.INTER_LINEAR)
        

            numDisparities = cv.getTrackbarPos('numDisparities','disp')*16
            blockSize = cv.getTrackbarPos('blockSize','disp')*2 + 1
            P1 = cv.getTrackbarPos('P1','disp') # * 3 * blockSize**2
            P2 = cv.getTrackbarPos('P2','disp') * 3 * blockSize**2
            preFilterCap = cv.getTrackbarPos('preFilterCap','disp')
            mode = cv.getTrackbarPos('mode','disp')
            uniquenessRatio = cv.getTrackbarPos('uniquenessRatio','disp')
            speckleRange = cv.getTrackbarPos('speckleRange','disp')
            speckleWindowSize = cv.getTrackbarPos('speckleWindowSize','disp')
            disp12MaxDiff = cv.getTrackbarPos('disp12MaxDiff','disp')
            minDisparity = cv.getTrackbarPos('minDisparity','disp')

            if P1 > P2:
                P2 = P1 + 3 * blockSize**2
            
            # Setting the updated parameters before computing disparity map
            stereo.setNumDisparities(numDisparities)
            stereo.setBlockSize(blockSize)
            stereo.setP1(P1)
            stereo.setP2(P2)
            stereo.setPreFilterCap(preFilterCap)
            stereo.setMode(mode)
            stereo.setUniquenessRatio(uniquenessRatio)
            stereo.setSpeckleRange(speckleRange)
            stereo.setSpeckleWindowSize(speckleWindowSize)
            stereo.setDisp12MaxDiff(disp12MaxDiff)
            stereo.setMinDisparity(minDisparity)

            tdisparity_start = perf_counter()
            rec_l_bw = cv.cvtColor(rectified_left, cv.COLOR_BGR2GRAY)
            rec_r_bw = cv.cvtColor(rectified_right, cv.COLOR_BGR2GRAY)
            
            disparity_map = (stereo.compute(rec_l_bw, rec_r_bw).astype(np.float32)/16)[:,minDisparity+numDisparities:]
            normalized_disparity_map = (((disparity_map - 4) * (255 / (160 - 4))).astype(np.uint8))
            tdisparity_end = perf_counter()

            # normalized_disparity_map = cv.normalize(disparity_map, None, 0, 255, cv.NORM_MINMAX).astype(np.uint8)
            cv.imshow('Disparity Map', normalized_disparity_map)

            image_3d = cv.reprojectImageTo3D(disparity_map, Qmat)
            image_3d_m = np.divide(image_3d, 1000)

            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = 'map'  # Adjust the frame_id as needed

            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
            ]

            point_cloud_msg = PointCloud2(
                header=header,
                fields=fields,
                height=disparity_map.shape[0],
                width=disparity_map.shape[1],
                is_dense=False,
                is_bigendian=False,
                point_step=12,
                row_step=12 * disparity_map.shape[1],
                data=b'',
            )

            reshaped_points = image_3d_m.reshape(-1, 3)
            packed_data = reshaped_points.astype(np.float32).tobytes()
            point_cloud_msg.data = packed_data
            # Publish the PointCloud2 message
            pub.publish(point_cloud_msg)

            broadcaster.sendTransform((0.0, 0.0, 0.0),(0, 0, 0, 1.0), rospy.Time.now(), "map", "/point_cloud")

            t1_stop = perf_counter()
            t1_duration = ((t1_stop - t1_start) * 1000)
            total_time += t1_duration

            tdisparity_duration = ((tdisparity_end - tdisparity_start) * 1000)
            disparity_time += tdisparity_duration

            # Display rectified images side by side
            cv.imshow("Left Rectified", rectified_left)
            cv.imshow("Right Rectified", rectified_right)

            # cv.imshow("Left Matched", rect_l_cpy)
            # cv.imshow("Kiwi Distances", rect_r_cpy)
            # cv.imshow("Kiwi Mask", opening)

            # Display the images and disparity map
            # cv.imshow("Left colour", color_left_image)
            # cv.imshow("Right colour", color_right_image)
            key = cv.waitKey(1)

        print(f"Average time: {total_time/loops:>5.2f}ms")
        print(f"Disparity time: {disparity_time/loops:>5.2f}ms")
    
    except rospy.ROSInterruptException:
        pass

    finally:
        # Stop grabbing images for both cameras
        camera_left.StopGrabbing()
        camera_right.StopGrabbing()
    
        # Close both cameras
        camera_left.Close()
        camera_right.Close()
    
        cv.destroyAllWindows()
        print("ending")

            

if __name__ == "__main__":
    main()
    