#!/usr/bin/env python3

#from ultralytics import YOLO
from ultralytics import YOLOv10
from pypylon import pylon
import cv2 as cv
import rospy
import os

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Polygon, Point32


import subprocess
import time
import signal



def start_process(executable_file):
    return subprocess.Popen(
        executable_file,
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )

def read_process(process):
    return process.stdout.readline().decode("utf-8").strip()

def stop_process(process):
    process.send_signal(signal.SIGINT)

def terminate(process):
    process.stdin.close()
    process.terminate()
    process.wait(timeout=0.2)



def scale_image(img, scale=100):
    width = int(img.shape[1] * scale / 100)
    height = int(img.shape[0] * scale / 100)
    dim = (width, height)
    img = cv.resize(img, dim, interpolation = cv.INTER_AREA)
    return img


cur_dir = os.path.dirname(os.path.abspath(__file__))
y10_path = os.path.join(cur_dir, 'best.pt')
engine_path = os.path.join(cur_dir, 'best.engine')

#model = YOLO('yoloV8_flowers.pt', task='detect')
model_y10 = YOLOv10(y10_path, task='detect')

#model.export(format="engine", imgsz= (480,640), half=True, int8=True, simplify=True, dynamic=True)
#model_y10.export(format="engine", imgsz= (480,640), half=True, int8=True, simplify=True, dynamic=True)

#tensorrt_model = YOLO("yoloV8_flowers.engine", task='detect')
tensorrt_model = YOLOv10(engine_path, task='detect')




def main():
    global polygons_pub
    process_path = os.path.join(cur_dir, 'processing_connect_test')

    print("start")

    rospy.init_node('image_detector', anonymous=True)

    image_sub = rospy.Subscriber("image_topic", Image, prediction_callback, queue_size=1)
    polygons_pub = rospy.Publisher('polygons_topic', Polygon, queue_size=1)

    print("begin subprocess")
    process = start_process(process_path)

    key = 0
    while not rospy.is_shutdown() and key != 27:
        key = cv.waitKey(1)
        #pass
    
    print("closing")
    stop_process(process)
    image_sub.unregister()
    cv.destroyAllWindows()
    print("Program ended")



def prediction_callback(data):
    if not hasattr(prediction_callback, "latency_counter"):
        prediction_callback.latency_counter = 0
        prediction_callback.latency_avg = 0
    global polygons_pub
    bridge = CvBridge()
    try:
        # Convert the ROS Image message to a OpenCV image
        cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

        send_time = data.header.stamp
        receive_time = rospy.Time.now()
        latency = round((receive_time - send_time).to_sec() * 1000, 3) 
        prediction_callback.latency_counter = prediction_callback.latency_counter + 1
        prediction_callback.latency_avg = prediction_callback.latency_avg + latency
        print(f"\rlatency: {latency:.3f} ms  |  Avg = {prediction_callback.latency_avg/prediction_callback.latency_counter:.3f} ms     ", end="")
        
        # Display the image using OpenCV
        cv.imshow("Received Image", cv_image)
        cv.waitKey(1)

        rectangles = run_prediction(tensorrt_model, cv_image)
        # print(points)

        polygon_msg = Polygon()

        for rect in rectangles:
            for corner in rect:
                point = Point32(x=corner[0], y=corner[1], z=0.0)
                polygon_msg.points.append(point)

        #print("sending rects")
        polygons_pub.publish(polygon_msg)


    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))
        print("error decoding image")
    


def run_prediction(model, image):
    results = model.predict(image, imgsz=(480,640), conf=0.4, iou=0.8, half=True, verbose=False)
    results = results[0] 
    regions = []
    for i in range(len(results.boxes)):
        box = results.boxes[i]
        tensor = box.xyxy[0]
        x1 = int(tensor[0].item())
        y1 = int(tensor[1].item())
        x2 = int(tensor[2].item())
        y2 = int(tensor[3].item())
        #cv.rectangle(image, (x1,y1), (x2,y2), (255,0,255), 3)
        regions.append(((x1,y1),(x2,y2)))
    # cv.imshow("Image", image)
    # cv.waitKey(1)
    return regions



if __name__ == "__main__":
    main()