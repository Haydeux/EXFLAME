#!/usr/bin/env python3


import cv2 as cv
import numpy as np

import time
import math
import threading

import rospy

from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped

import rtdeState

# Real Sense
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge
from std_msgs.msg import Header


# Defines the interface to launch the baslers C++ program
class baslers:
    def __init__(self):
        self.on = False

    def turn_on(self):
        self.on = True
    
    def turn_off(self): 
        self.on = False



# Defining the ur five input and output communication
class ur_five:
    ### BASIC INTERFACING ###
    # Turn on the ur_five object
    def turn_on(self):
        self.on = True
        
        # Connecting to the realtime data exchange
        self.rtde = rtdeState.RtdeState('192.168.10.20', 'rtdeCommand.xml')
        self.rtde.initialize()

        # Start the stream that publishes the position of the target
        self.position_pub = rospy.Publisher("flame/perceived_position", PointStamped, queue_size = 1)
        self.error_pub = rospy.Publisher("flame/position_error", Point, queue_size = 1)

        # Starting the workflow of predicted position to RTDE
        self.target_sub = rospy.Subscriber("/flame/predicted_position", Point, self.upload_pose)
        self.fruits_sub = rospy.Subscriber("/flame/realsense_positions", PoseArray, self.servo_realsense)

        # Start the thread to record the TCP poses of the UR5
        tcp_recorder = threading.Thread(target = self.record_tcp)
        tcp_recorder.start()

    # Turn off the ur_five object
    def turn_off(self):
        # Disconnecting all the cables
        self.position_pub.unregister()
        self.error_pub.unregister()
        self.target_sub.unregister()
        self.fruits_sub.unregister()
        
        self.on = False

    ### INITIALISATION ###
    # Starting up the global variables
    def __init__(self, fruit = True):
        # Starting the object as off
        self.on = False

        # Declaring the rate at which the robot can run
        self.target_time = time.time()
        self.delay_time = 0.0

        # Creating a shift register to store a large amount of previous TCPs
        self.tcps = [0] * 100

        # Creating variables to store the active poses of the objects
        self.actual_tcp = [0, 0, 0, 0, 0, 0]

        # The offset of the RGB realsense camera and the TCP
        self.realsense_offset = [-0.215, 0.0, -0.055]

        # Defining the desired distance from the TCP for fruit and flowers
        if fruit:
            self.tcp_offset = [0, 0, 0.230]
        else:
            self.tcp_offset = [0, 0, 0.260]

        # Defining a variable to track the state according to the UR5 or the computer
        self.pick_state = 0

    # The function that continuously records the TCP pose of the UR5 when it is on
    def record_tcp(self) -> None:
        while self.on:
            self.download_pose()
            time.sleep(0.01)

    ### RTDE COMMUNICATION ###
    # Appends the current pose of the UR5 to the array of poses
    def download_pose(self) -> None:
        # Update the state of the robot
        state = self.rtde.receive()
        time_stamp = time.time()

        tcp = [0, 0, 0, 0, 0, 0]

        # Read the current position of the UR5
        tcp[0] = state.actual_TCP_pose[0]
        tcp[1] = state.actual_TCP_pose[1]
        tcp[2] = state.actual_TCP_pose[2]
        tcp[3] = state.actual_TCP_pose[3]
        tcp[4] = state.actual_TCP_pose[4]
        tcp[5] = state.actual_TCP_pose[5]

        # Shift all the values across the register
        self.tcps[1:] = self.tcps[:-1]

        # Append the most recent reading to the start of the list
        self.tcps[0] = [tcp, time_stamp]

    # Uploads a pose to the registers used for communicating pose with the RTDE
    def upload_pose(self, position: Point) -> None:
        target = [0, 0, 0, 0, 0, 0]

        # Dealing with the translation first
        target[0] = position.x
        target[1] = position.y
        target[2] = position.z

        # If any of the movement limits are violated do not upload the target
        if (position.x < 0.2) or (position.x > 0.84):
            print("Position out of range x")
            return
        if (position.y < -0.6) or (position.y > 0.6):
            print("Position out of range y")
            return
        if position.z > 0.64:
            print("Position out of range z")
            print(position.z)
            return
        
        # Dealing with the rotation next
        rotation = self.get_rotation()

        # If any of the movement limits are violated do not upload the target
        if (rotation.x < -0.6) or (rotation.x > 0.6):
            print("Rotation out of range x")
            return
        if (rotation.y < -0.6) or (rotation.y > 0.6):
            print("Rotation out of range y")
            return
        if (rotation.y < -0.6) or (rotation.y > 0.6):
            print("Rotation out of range z")
            return

        target[3] = rotation.x
        target[4] = rotation.y
        target[5] = rotation.z

        # If time greater or equal to the delay time has passed, upload the target position
        if time.time() >= self.target_time:
            # Upload the target to the UR5 robotic arm
            for i in range(0, len(target)):
                self.rtde.target_point.__dict__["input_double_register_%i" % i] = target[i]
            self.rtde.con.send(self.rtde.target_point)

            # Set the next time which a message should be sent
            self.target_time = time.time() + self.delay_time

    # Updates all the variables of the code to match that of the actual UR5
    def download_variables(self) -> None:
        state = self.rtde.receive()

        # Read the state variable from the UR5
        self.pick_state = state.output_int_register_0

    # Either enable or disable the servoing variable on the robot
    def upload_variables(self, state: int) -> None:
        # Signal to the UR5 that it may execute a servoing command
        self.rtde.servo.input_int_register_0 = state
        self.rtde.con.send(self.rtde.servo)

    ### ROSTOPIC COMMUNICATION ###
    # Publishes the target to the predictive motion node
    def publish_position(self, position: Point, position_time: float) -> None:
        # If the position is empty dont try to send it
        if position == None:
            return

        # Publish the global position of the detected fruit for data recording
        pub_point = PointStamped()

        pub_point.point.x = position.x
        pub_point.point.y = position.y
        pub_point.point.z = position.z

        pub_point.header.stamp.secs = int(position_time)
        pub_point.header.stamp.nsecs = int((position_time % 1) * 10 ** 9)

        self.position_pub.publish(pub_point)

        # Calculates the error of the current target and publishes it to the ros topic
        error = Point()

        error.x = position.x - self.actual_tcp[0]
        error.y = position.y - self.actual_tcp[1]
        error.z = position.z - self.actual_tcp[2]

        self.error_pub.publish(error)

    ### POSITION MANIPULATION ###
    # Converts a coordinate relative to the TCP to a coordinate relative to the base
    def tcp_to_base(self, point: Point) -> Point:
        # If no TCP pose has been sent yet, do not attempt to perform the calculation
        if self.actual_tcp == [0,0,0,0,0,0]:
            return

        # First rotating the position of the fruit relative to the global axes
        # Note that the angles of x and y are flipped to counteract some flipping action done earlier
        rx = - self.actual_tcp[3]
        ry = - self.actual_tcp[4]
        rz = self.actual_tcp[5]

        # Converting the pose to axis angle notation
        theta = math.sqrt(rx * rx + ry * ry + rz * rz)

        ux = rx / theta
        uy = ry / theta
        uz = rz / theta

        # Converting the axis angle to a rotation matrix per term for x, y and z 
        # Refer to the Wikipedia article on rotation matrix: rotation matrix from axis angle

        # Precalculating the sine and cosine for readability and efficiency
        cos = math.cos(theta)
        sin = math.sin(theta)

        # The notation here refers to the row coordinate then the column
        # For example xy refers to the proportion of old y in the new x
        xx = cos + ux * ux * (1 - cos)
        xy = uy * ux * (1 - cos) + uz * sin
        xz = uz * ux * (1 - cos) - uy * sin

        yx = ux * uy * (1 - cos) - uz * sin
        yy = cos + uy * uy * (1 - cos)
        yz = uz * uy * (1 - cos) + ux * sin

        zx = ux * uz * (1 - cos) + uy * sin
        zy = uy * uz * (1 - cos) - ux * sin
        zz = cos + uz * uz * (1 - cos)

        # Multiplying the per term rotation by the inputted point
        rotated_point = Point()

        rotated_point.x = point.x * xx + point.y * xy + point.z * xz
        rotated_point.y = point.x * yx + point.y * yy + point.z * yz
        rotated_point.z = point.x * zx + point.y * zy + point.z * zz

        # Secondly applying the offset of the TCP position
        transformed_point = Point()

        transformed_point.x = rotated_point.x + self.actual_tcp[0]
        transformed_point.y = rotated_point.y + self.actual_tcp[1]
        transformed_point.z = rotated_point.z + self.actual_tcp[2]

        # print("base x: " + str(round(transformed_point.x, 2)), end="")
        # print(" y: " + str(round(transformed_point.y, 2)), end="")
        # print(" z: " + str(round(transformed_point.z, 2)))

        return transformed_point

    # Converts a coordinate given relative to the realsense to a coordinate relative to the TCP
    def realsense_to_tcp(self, point: Point) -> Point:
        # Applying a rotation of the realsense relative to the TCP
        # Converting the rotation angle to radians
        theta = 31.5 * math.pi / 180

        # Flipping the direction of the x and y axes
        x_1 = - point.x
        y_1 = - point.y
        z_1 = point.z

        # Rotating the x and z axes
        x_2 = x_1 * math.cos(theta) - z_1 * math.sin(theta)
        y_2 = y_1
        z_2 = x_1 * math.sin(theta) + z_1 * math.cos(theta)

        # Applying a translation of the realsense relative to the UR5 TCP
        x_3 = x_2 - self.realsense_offset[0]
        y_3 = y_2 - self.realsense_offset[1]
        z_3 = z_2 - self.realsense_offset[2]

        # Applying a translation to account for the desired TCP offset
        transformed_point = Point()

        transformed_point.x = x_3 - self.tcp_offset[0]
        transformed_point.y = y_3 - self.tcp_offset[1]
        transformed_point.z = z_3 - self.tcp_offset[2]

        # print("tcp x: " + str(round(transformed_point.x, 2)), end="")
        # print(" y: " + str(round(transformed_point.y, 2)), end="")
        # print(" z: " + str(round(transformed_point.z, 2)))

        return transformed_point

    # Takes the position relative to the nerian and converts it to global coordinates
    def nerian_to_base(self, point: Point) -> Point:
        position = Point()
        
        position.x = - point.y + 0.6
        position.y = point.x - 0.025
        position.z = point.z - 0.65

        return position
    
    ### MISCELLANEOUS ###
    # Given a list of candidate targets relative to the realsense determines the position of the closest target and the global coordinates
    def servo_realsense(self, targets: PoseArray) -> None:
        # Declaring the time when the image was captured
        image_time = float(targets.header.stamp.secs) + float(targets.header.stamp.nsecs) / (10 ** 9)

        # Updates the pose values of the UR5 and realsense
        self.update_pose(image_time)

        min_error = 1
        
        # Convert all the targets to TCP coordinates and evaluate the best one according to distance from the TCP
        for target in targets.poses:
            # Comparing the distance from the TCP to the fruit
            position = self.realsense_to_tcp(target.position)
            error = abs(position.x) + abs(position.y) + abs(position.z)

            # Calculating if this is a minimum reading and saving it if it is
            if error < min_error:
                best = position
                min_error = error

        # Calculating the global position of the kiwifruit
        best_position = self.tcp_to_base(best)

        # Publish the closest position to the motion estimator
        self.publish_position(best_position, image_time)

    # Returns the rotation value of the UR5 as a point
    def get_rotation(self) -> Point:
        rotation = Point()

        # Approximate the rotation about the y axis to be a sine wave
        rotation.x = 0
        rotation.y = 0 # 0.5 * math.sin(0.5 * (2 * math.pi * time.time()))
        rotation.z = 0

        return rotation

    # Given a certain time stamp it will set the current tcp pose to that of the nearest time match
    def update_pose(self, pose_time) -> None:
        # Iterate through all the tcp readings until one is later than the desired time
        for tcp in self.tcps:
            if (tcp != 0):
                if (tcp[1] < pose_time):
                    self.actual_tcp = tcp[0]
                    break


# Defining the camera in hand input and image processing object
class realsense:
    ### BASIC INTERFACING ###
    # Connecting up all the internals and turning it on
    def turn_on(self):
        # Waiting for camera intrinsics to be sent
        K_matrix = rospy.wait_for_message('/camera/aligned_depth_to_color/camera_info', CameraInfo).K

        # Storing the entrinsics for future calculations
        self.x_focal = K_matrix[0]
        self.y_focal = K_matrix[4]
        self.x_centre = K_matrix[2]
        self.y_centre = K_matrix[5]

        # Starting up all the ROS subscribers of the image streams
        self.depth_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.process_depth, queue_size = 1)
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.process_image, queue_size = 1)

        # Starting up all the ROS publishers for outputing the kiwifruit point
        self.position_pub = rospy.Publisher("/flame/realsense_positions", PoseArray, queue_size = 1)

        self.on = True

    # Shutting it down elegantly 
    def turn_off(self):
        # Disconnecting all the cables
        self.depth_sub.unregister()
        self.image_sub.unregister()

        self.on = False

    ### INITIALISATION ###
    # Initialise the default values of the HSV filter
    def __init__(self, fruit = True):
        # Initialising the image processing values
        self.bridge = CvBridge()

        # Changing the way that the realsense perceives position depending on target type
        if fruit:
            self.target_depth = 25
            self.hsv_ranges = [(15, 80, 0), (25, 255, 255)]
        else:
            self.target_depth = 0
            self.hsv_ranges = [(20, 80, 40), (35, 255, 255)]

        # The expected delay of capturing an image
        self.delay = 0.034

    # Load in the YOLO model
    def load_yolo(self):
        # Importing the machine learning stuff
        from ultralytics import YOLO

        self.fruit_detector = YOLO("flame_fake_kiwifruit.pt")

    ### IMAGE PROCESSING ###
    # Extracting a fruit image positions using thresholding
    def extract_fruit(self, frame):
        # Converting the image to HSV and extracting kiwifruit
        frame_hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        frame_mask = cv.inRange(frame_hsv, self.hsv_ranges[0], self.hsv_ranges[1])

        # cv.imshow("one", frame_mask)
        # cv.waitKey(1)

        # Applying some morphological filters
        kernel_open = cv.getStructuringElement(cv.MORPH_ELLIPSE, [5, 5])
        kernel_close = cv.getStructuringElement(cv.MORPH_ELLIPSE, [23, 23])

        frame_mask = cv.morphologyEx(frame_mask, cv.MORPH_OPEN, kernel_open)
        frame_mask = cv.morphologyEx(frame_mask, cv.MORPH_CLOSE, kernel_close)

        # Finding the contours of the potential fruit
        contours, heirarchy = cv.findContours(frame_mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        frame_mask = cv.drawContours(frame_mask, contours, -1, 127, 3)

        fruits = []

        # Test if the contours are large enough
        for contour in contours:
            if cv.contourArea(contour) > 400:
                # Do position calculations if it is
                m = cv.moments(contour)
                cx = int(m["m10"] / m["m00"])
                cy = int(m["m01"] / m["m00"])

                fruits.append([cx, cy])

        return fruits

    # Extract the fruit from the target image except using YOLO
    def extract_fruit_yolo(self, frame):
        results = self.fruit_detector.predict(frame, save=False, show=False, conf=0.2, imgsz = (448, 256), boxes = False)

        fruits = []

        for fruit in results[0].boxes:
            cx = int((fruit.xyxy[0][2] + fruit.xyxy[0][0]) / 2)
            cy = int((fruit.xyxy[0][3] + fruit.xyxy[0][1]) / 2)

        fruits.append([cx, cy])

        return fruits

    # Converting an image position to a cartesian position
    def image_to_realsense(self, image_position) -> Point:
        # Returning the error for the control system to deal with
        position = Point()

        # Doing the calculations to locate the 3D position of a given pixel on the image
        position.z = 0.001 * self.get_depth(image_position)
        position.x = 0.001 * float((image_position[0] - self.x_centre) * position.z * 1000) / self.x_focal
        position.y = 0.001 * float((image_position[1] - self.y_centre) * position.z * 1000) / self.y_focal

        return position
    
    # Given a point on the image the average depth of the neighborhood will be found and offset applied
    def get_depth(self, image_position) -> float:
        # Defining the boundaries of the area of extraction
        sample_radius = 5

        x_min = image_position[1] - sample_radius
        x_max = image_position[1] + sample_radius
        y_min = image_position[0] - sample_radius
        y_max = image_position[0] + sample_radius

        # Extracting and copying a data segment from the current depth image
        depth_segment = np.copy(self.depth_image[x_min:x_max, y_min:y_max])

        # Finding the number of valid readings in the depth segment
        num_readings = sum(sum(sum([depth_segment != 0])))
        
        # Getting the average reading of the depth segment excluding zeros
        depth_reading = sum(sum(depth_segment)) / num_readings

        # Adding on 25 mm to get the approximate centre of the kiwifruit
        depth_reading += self.target_depth

        return depth_reading
  
    ### CALLBACKS ###
    # Save the most recent depth image into a variable
    def process_depth(self, data):
        self.depth_image = self.bridge.imgmsg_to_cv2(data)

    # Processing an RGB image when received
    def process_image(self, data):
        # Recording the time prior to any preprocessing
        image_time = time.time() - self.delay

        # Formatting the time for the header object
        header = Header()
        header.stamp.secs = int(image_time)
        header.stamp.nsecs = int((image_time % 1) * 10 ** 9)

        # Converting the received image and extracting the fruit from it
        image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        fruits = self.extract_fruit(image)

        # Converting the image position of the fruit to a spacial position
        fruit_poses = []

        if fruits is not None:
            for fruit in fruits:
                # Displaying the fruit on the image
                cv.circle(image, (fruit[0], fruit[1]), 3, (255, 0, 255), -1)

                # Saving the fruit poses
                fruit_pose = Pose()
                fruit_pose.position = self.image_to_realsense(fruit)
                fruit_poses.append(fruit_pose)

        cv.imshow("window", image)
        cv.waitKey(1)

        fruit = PoseArray()
        fruit.header = header
        fruit.poses = fruit_poses

        # Publishing all the fruit poses that were identified
        self.position_pub.publish(fruit)
  


# END