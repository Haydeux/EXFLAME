#!/usr/bin/env python3

#from ultralytics import YOLO
from ultralytics import YOLOv10
from pypylon import pylon
import cv2 as cv
import rospy
import os

from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image


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

    rospy.init_node('image_listener', anonymous=True)

    while not rospy.is_shutdown():

        # ros recieve
        image_sub = rospy.Subscriber("image_topic", Image, prediction_callback)


        # ros send


def prediction_callback(data):
    bridge = CvBridge()
    try:
        # Convert the ROS Image message to a OpenCV image
        cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        
        # Display the image using OpenCV
        cv.imshow("Received Image", cv_image)
        cv.waitKey(1)

        points = run_prediction(tensorrt_model, cv_image)
        print(points)
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))
        print("error decoding image")
    


def run_prediction(model, image):
    results = model.predict(image, imgsz=(480,640), conf=0.25, iou=0.45, half=True, verbose=False)
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
    return regions



if __name__ == "__main__":
    main()