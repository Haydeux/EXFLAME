#!/usr/bin/env python3

import cv2 as cv
import numpy as np

import os
import subprocess
import signal
import time
import math
import threading

import Jetson.GPIO as GPIO

import rospy

from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64

import rtdeState

import csv

import random


# Defines the interface to launch the baslers C++ program
class baslers:
    def __init__(self):
        self.program_dir = "/home/geri/Documents/EXFLAME/Git_clone/EXFLAME/2024/detection_connect_test.py"
        self.on = False

    def turn_on(self):
        self.process = subprocess.run(['gnome-terminal', '--', '/usr/bin/python3', self.program_dir])
        #time.sleep(10)
        self.on = True
    
    def turn_off(self):
        # try:
        #     #self.process.send_signal(signal.SIGINT)
        #     ps_output = subprocess.check_output(['ps', 'aux'])
        #     lines = ps_output.decode().split('\n')
        #     for line in lines:
        #         if 'python3 script2.py' in line:
        #             pid = int(line.split()[1])
        #             os.kill(pid, signal.SIGINT)
        #             break
        # except:
        #     print("already off") 
        self.on = False



class laser:    
    def __init__(self):
        # Disable warnings
        #GPIO.setwarnings(False)

        # Pin Definitions
        self.laser_pin = 12  # BCM pin 18, BOARD pin 12

        # Set up the GPIO channel
        GPIO.setmode(GPIO.BOARD)  # Use BOARD pin numbering
        GPIO.setup(self.laser_pin, GPIO.OUT, initial=GPIO.LOW)  # Set pin as output and initialize to low

        self.change_time = time.time()
        self.laser_state_on = False

        self.on = False

    def turn_on(self):
        self.change_time = time.time()
        self.error_sub = rospy.Subscriber("xflame/position_error", Point, self.operate_laser)
        self.on = True
    
    def turn_off(self):
        self.laser_off()
        GPIO.cleanup()  # Clean up GPIO on exit         
        self.error_sub.unregister()
        self.on = False

    def laser_on(self):
        GPIO.output(self.laser_pin, GPIO.HIGH)
        self.laser_state_on = True

    def laser_off(self):
        GPIO.output(self.laser_pin, GPIO.LOW)
        self.laser_state_on = False

    def operate_laser(self, position:PointStamped):
        distance = np.sqrt(np.power(position.x, 2) + np.power(position.y, 2) + np.power(position.z, 2))
        
        if ((time.time() - self.change_time) > 1.0):
            if ((distance <= 0.01) and (not self.laser_state_on)):
                self.laser_on()
            elif ((distance > 0.01) and (self.laser_state_on)):
                self.laser_off()



# Defining the ur five input and output communication
class ur_five:
    ### BASIC INTERFACING ###
    # Turn on the ur_five object
    def turn_on(self):
        self.on = True
        
        # Connecting to the realtime data exchange
        self.rtde = rtdeState.RtdeState('192.168.10.20', '/home/geri/Documents/EXFLAME/Git_clone/EXFLAME/ex_flame_ROS_package/rtdeCommand.xml')
        self.rtde.initialize()

        # Start the stream that publishes the position of the target
        self.position_pub = rospy.Publisher("xflame/perceived_position", PointStamped, queue_size = 1)
        self.error_pub = rospy.Publisher("xflame/position_error", Point, queue_size = 1)

        # Starting the workflow of predicted position to RTDE
        self.target_sub = rospy.Subscriber("xflame/predicted_position", PointStamped, self.upload_pose)
        self.fruits_sub = rospy.Subscriber("xflame/baslers_positions", PoseArray, self.servo_baslers)

        # Start the thread to record the TCP poses of the UR5
        tcp_recorder = threading.Thread(target = self.record_tcp)
        tcp_recorder.start()

    # Turn off the ur_five object
    def turn_off(self):
        # Disconnecting all the cables
        #try:
        self.clear_pose()
        self.position_pub.unregister()
        self.error_pub.unregister()
        self.target_sub.unregister()
        self.fruits_sub.unregister()
        #except:
        #print("already off")
        
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

        # The offset of the camera and the TCP
        self.realsense_offset = [-0.215, 0.0, -0.055]
        self.baslers_offset = [0.067, 0.0175, 0.0425]
        self.baslers_rotation = [[ 0, -1,  0],
                                 [-1,  0,  0],
                                 [ 0,  0,  1]]

        # Defining the desired distance from the TCP for fruit and flowers
        if fruit:
            self.tcp_offset = [0, 0, 0.230]
        else:
            self.tcp_offset = [0.0325, 0.00, 0.350] #[0.035, 0.00, 0.400] #[0.06, 0.02, 0.400]

        # Defining a variable to track the state according to the UR5 or the computer
        self.pick_state = 0

        # For timing
        self.total_mean_time_pub = rospy.Publisher('times/total/mean', Float64, queue_size=1)
        self.total_std_time_pub = rospy.Publisher('times/total/std', Float64, queue_size=1)
        self.upload_mean_time_pub = rospy.Publisher('times/upload/mean', Float64, queue_size=1)
        self.upload_std_time_pub = rospy.Publisher('times/upload/std', Float64, queue_size=1)
        self.conversion_mean_time_pub = rospy.Publisher('times/conversion/mean', Float64, queue_size=1)
        self.conversion_std_time_pub = rospy.Publisher('times/conversion/std', Float64, queue_size=1)

        self.total_times = []
        self.upload_times = []
        self.conversion_times = []

        self.loops_t = -100
        self.loops_u = -100
        self.loops_c = -100

        self.print_t = True
        self.print_u = True
        self.print_c = True

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
    def upload_pose(self, position_stamped: PointStamped) -> None:
        # Start timing
        start_time = time.time()

        target = [0, 0, 0, 0, 0, 0]

        # Applying a translation to account for the desired TCP offset
        position = Point()
        position.x = position_stamped.point.x - self.tcp_offset[0]
        position.y = position_stamped.point.y - self.tcp_offset[1]
        position.z = position_stamped.point.z - self.tcp_offset[2]

        # position = self.baslers_to_tcp(position)
        # position = self.tcp_to_base(position)

        # Dealing with the translation first
        target[0] = position.x
        target[1] = position.y
        target[2] = position.z

        # If any of the movement limits are violated do not upload the target
        if (position.x < 0.2) or (position.x > 0.84): #change?
            print("Position out of range x")
            return
        if (position.y < -0.6) or (position.y > 0.6): #change to 0.7?
            print("Position out of range y")
            return
        if position.z > 0.7: #change to 0.75?
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
        if (rotation.z < -0.6) or (rotation.z > 0.6):
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

        # Measure time
        end_time = time.time()
        capture_time = position_stamped.header.stamp.to_time()
        total_elapse_time = (end_time - capture_time) * 1000
        elapse_time = (end_time - start_time) * 1000 # measure in ms

        # self.n += 1
        # if self.n > 0:
        #     self.avg_time = (self.avg_time * (self.n - 1) + total_elapse_time) / self.n
        #     #print(f"{elapse_time:.2f} ms    |    {self.avg_time:.2f} ms           ", end="\r")

        # Logic to record and display times, for just the servo baslers section
        if self.loops_u < 0:
            self.loops_u += 1
        elif self.loops_u > 5000:
            if self.print_u:
                # Upload times
                mean_time = np.mean(self.upload_times, dtype=np.float64)
                std_time = np.std(self.upload_times, dtype=np.float64)
                self.upload_mean_time_pub.publish(mean_time)
                self.upload_std_time_pub.publish(std_time)

                # Total times
                mean_t_time = np.mean(self.total_times, dtype=np.float64)
                std_t_time = np.std(self.total_times, dtype=np.float64)
                self.total_mean_time_pub.publish(mean_t_time)
                self.total_std_time_pub.publish(std_t_time)
            self.print_u = False
        else:
            self.total_times.append(total_elapse_time)
            self.upload_times.append(elapse_time)
            self.loops_u += 1

    # Uploads a pose to the registers used for communicating pose with the RTDE
    def clear_pose(self) -> None:
        target = [0, 0, 0, 0, 0, 0]

        # Upload the target to the UR5 robotic arm
        for i in range(0, len(target)):
            self.rtde.target_point.__dict__["input_double_register_%i" % i] = target[i]
        self.rtde.con.send(self.rtde.target_point)


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
    def publish_position(self, position: PointStamped, position_time: float) -> None:
        # If the position is empty dont try to send it
        if position == None:
            return

        # Publish the global position of the detected fruit for data recording
        pub_point = PointStamped()

        pub_point.point.x = position.point.x
        pub_point.point.y = position.point.y
        pub_point.point.z = position.point.z

        pub_point.header.stamp = position.header.stamp

        # Calculates the error of the current target and publishes it to the ros topic
        error = Point()

        error.x = position.point.x - self.tcp_offset[0] - self.actual_tcp[0]
        error.y = position.point.y - self.tcp_offset[1] - self.actual_tcp[1]
        error.z = position.point.z - self.tcp_offset[2] - self.actual_tcp[2]

        # Publish ROS messages
        self.position_pub.publish(pub_point)
        self.error_pub.publish(error)

    ### POSITION MANIPULATION ###
    # Converts a coordinate relative to the TCP to a coordinate relative to the base
    def tcp_to_base(self, point: Point) -> Point:
        # If no TCP pose has been sent yet, do not attempt to perform the calculation
        if self.actual_tcp == [0,0,0,0,0,0]:
            return

        # First rotating the position of the fruit relative to the global axes
        # Note that the angles of x and y are flipped to counteract some flipping action done earlier
        # rx = - self.actual_tcp[3]
        # ry = - self.actual_tcp[4]
        rx = self.actual_tcp[3]
        ry = self.actual_tcp[4]
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

    # Converts a coordinate given relative to the baslers to a coordinate relative to the TCP
    def baslers_to_tcp(self, point: Point) -> Point:
        # Apply the baslers rotation matrix to the point
        rotated_point = np.dot(np.array(self.baslers_rotation), np.array([point.x, point.y, point.z]))
        
        # Apply the baslers translation offset to the point
        translated_point = rotated_point + np.array(self.baslers_offset)

        transformed_point = Point()
        transformed_point.x = translated_point[0]
        transformed_point.y = translated_point[1]
        transformed_point.z = translated_point[2]

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

        best_pose = PointStamped()
        best_pose.point.x = best_position.x
        best_pose.point.y = best_position.y
        best_pose.point.z = best_position.z
        best_pose.header.stamp = targets.header.stamp

        # Publish the closest position to the motion estimator
        self.publish_position(best_pose, image_time)

    # Given a list of candidate targets relative to the baslers, determines the position of the closest target and the global coordinates
    def servo_baslers(self, targets: PoseArray) -> None:
        # Start timing here
        start_time = time.time()

        # Declaring the time when the image was captured
        image_time = float(targets.header.stamp.secs) + float(targets.header.stamp.nsecs) / (10 ** 9)

        # Updates the pose values of the UR5 and realsense
        self.update_pose(image_time)
        
        best = None
        min_error = 1000
        
        # Convert all the targets to TCP coordinates and evaluate the best one according to distance from the TCP
        for target in targets.poses:
            # Comparing the distance from the TCP to the fruit
            position = self.baslers_to_tcp(target.position)
            error = abs(position.x) + abs(position.y) + abs(position.z)

            # Calculating if this is a minimum reading and saving it if it is
            if error < min_error:
                best = position
                #best_position = target.position
                min_error = error

        # Calculating the global position of the kiwifruit
        best_position = self.tcp_to_base(best)

        best_pose = PointStamped()
        best_pose.point = best_position
        best_pose.header.stamp = targets.header.stamp

        # Publish the closest position to the motion estimator
        self.publish_position(best_pose, image_time)

        # End timing here
        end_time = time.time()
        elapse_time = (end_time - start_time) * 1000 # measure in ms

        # Logic to record and display times, for just the servo baslers section
        if self.loops_c < 0:
            self.loops_c += 1
        elif self.loops_c > 5000:
            if self.print_c:
                mean_time = np.mean(self.conversion_times, dtype=np.float64)
                std_time = np.std(self.conversion_times, dtype=np.float64)
                self.conversion_mean_time_pub.publish(mean_time)
                self.conversion_std_time_pub.publish(std_time)
            self.print_c = False
        else:
            self.conversion_times.append(elapse_time)
            self.loops_c += 1

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
                #print(f"TCP time: {tcp[1]:.4f}  |  Pose time: {pose_time:.4f}")
                if (tcp[1] < pose_time):
                    self.actual_tcp = tcp[0]
                    #print("time chosen")
                    break




# Predicts flower motion and future position
class motion_estimator:
    ### BASIC INTERFACING ###
    # Turning on and connecting up all the components
    def turn_on(self):
        # Starting up the input and output nodes of the system
        self.point_pub = rospy.Publisher("xflame/predicted_position", PointStamped, queue_size = 1)
        self.point_sub = rospy.Subscriber("xflame/perceived_position", PointStamped, self.predict_position)

        self.on = True

    # Closing down the system cleanly
    def turn_off(self):
        # Disconnecting all the cables
        #try:
        self.point_pub.unregister()
        self.point_sub.unregister()
        #except:
        #print("already off")    

        self.on = False

    # Initialising the crystal ball
    def __init__(self):
        self.on = False

        self.position_list = [["Time in ms", "X in mm", "Y in mm", "Z in mm"]]
        self.avg_time = 0
        self.n = -10

        self.record_start = None
        self.recorded = False

        self.last_move = time.time()
        self.wait_time = -1

        self.targ_x = -1
        self.targ_y = -1
        self.targ_z = 0.85

        self.options = [[0.295, 0.403], [0.394, 0.202]]# x: 0.2950787079890232
        # y: 0.40329630659396787
        # z: 0.8793573860203547

        self.num = 0

        # x: 0.39358840607362866
        # y: 0.20203002552276275
        # z: 0.836350889356388

 
    # Callback for recieving fruit positions
    def predict_position(self, data: PointStamped):
        fruit_position = data.point

        # current_time = rospy.Time.now().to_time()
        # capture_time = data.header.stamp.to_time()
    
        # elapse_time = (current_time - capture_time) * 1000

        # self.n += 1
        # if self.n > 0:
        #     if self.record_start is None:
        #         self.record_start = capture_time

        #     if len(self.position_list) < 100:
        #         pos_time = [round((capture_time - self.record_start)*1000,5), round(fruit_position.x*1000,2), round(fruit_position.y*1000,2), round(fruit_position.z*1000,2)]
        #         self.position_list.append(pos_time)
        #         print(f"recording... {len(self.position_list)}      ",end="\r")
        #     elif not self.recorded:
        #         with open("data.csv", "w", newline="") as file:
        #             writer = csv.writer(file)
        #             writer.writerows(self.position_list)
        #         print("\nwritten to csv")
        #         self.recorded = True
        #     else:
        #         print(f"{elapse_time:.2f} ms    |    {self.avg_time:.2f} ms           ", end="\r")
        #         pass


        #     self.avg_time = (self.avg_time * (self.n - 1) + elapse_time) / self.n
        #     #print(f"{elapse_time:.2f} ms    |    {self.avg_time:.2f} ms           ", end="\r")
        
        predicted_point = PointStamped()
        predicted_point.header.stamp = data.header.stamp
        predicted_point.point.x = fruit_position.x 
        predicted_point.point.y = fruit_position.y 
        predicted_point.point.z = fruit_position.z 


        if time.time() - self.last_move >= 10:
            if self.wait_time == -1:
                self.num += 1
                if self.num > 1:
                    self.num = 0
                self.targ_x = self.options[self.num][0]
                self.targ_y = self.options[self.num][1]
                self.targ_z = 0.85  
                
                self.wait_time = time.time()
            else:
                if time.time() - self.wait_time >= 1.5:
                    self.wait_time = -1
                    self.last_move = time.time()
            predicted_point.point.x = self.targ_x 
            predicted_point.point.y = self.targ_y 
            predicted_point.point.z = self.targ_z 

        # x: 0.2950787079890232
        # y: 0.40329630659396787
        # z: 0.8793573860203547

        # x: 0.39358840607362866
        # y: 0.20203002552276275
        # z: 0.836350889356388

        # print(predicted_point.point)

        self.point_pub.publish(predicted_point)



# END OF FILE