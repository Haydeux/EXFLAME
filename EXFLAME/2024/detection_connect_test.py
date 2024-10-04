#!/usr/bin/env python3

from ultralytics import YOLO
from ultralytics import YOLOv10
from pypylon import pylon
import cv2 as cv
import numpy as np
import rospy
import os

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Polygon, Point32
from std_msgs.msg import Float64


import subprocess
import time
import signal
import sys


yolo_frame = 0


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
print(cur_dir)
y10_path = os.path.join(cur_dir, 'best.pt')
engine_path = os.path.join(cur_dir, 'best.engine')



#model_y10 = YOLOv10(y10_path, task='detect')
# model_y10.export(format="engine", imgsz= (480,640), half=True, int8=True, simplify=True, dynamic=True)
tensorrt_model = YOLOv10(engine_path, task='detect')


#model = YOLO('yoloV8_flowers.pt', task='detect')
#model.export(format="engine", imgsz= (480,640), half=True, int8=True, simplify=True, dynamic=True)
#tensorrt_model = YOLO("yoloV8_flowers.engine", task='detect')


def main():
    global polygons_pub, mean_time_pub, std_time_pub
    #process_path = os.path.join(cur_dir, 'processing_connect') #_test')
    process_path = os.path.join(cur_dir, 'processing_connect_timing')

    print("warming up")
    warm_up_image_path = os.path.join(cur_dir, 'warm_up.png')
    warm_up_image = cv.imread(warm_up_image_path)
    #run_prediction(tensorrt_model, warm_up_image)
    run_prediction(tensorrt_model, warm_up_image)

    print("start")

    rospy.init_node('image_detector', anonymous=True)

    image_sub = rospy.Subscriber("xflame/baslers_image", Image, prediction_callback, queue_size=1)
    polygons_pub = rospy.Publisher('xflame/bounding_boxes', Polygon, queue_size=1)

    mean_time_pub = rospy.Publisher('times/detection/mean', Float64, queue_size=1)
    std_time_pub = rospy.Publisher('times/detection/std', Float64, queue_size=1)

    print("begin subprocess")
    process = start_process(process_path)

    #key = 0
    # while not rospy.is_shutdown(): #and key != 27:
    #     #key = cv.waitKey(1)
    #     pass
    rospy.spin()
    
    print("closing")
    stop_process(process)
    image_sub.unregister()
    polygons_pub.unregister()
    #cv.destroyAllWindows()
    print("Program ended")


def prediction_callback(data):
    # if not hasattr(prediction_callback, "latency_counter"):
    #     prediction_callback.latency_counter = -20 * (yolo_frame+1)
    #     prediction_callback.latency_avg = 0
    #     prediction_callback.ml_times = []
    #     prediction_callback.pm_times = []
    #     prediction_callback.times = []
    #     prediction_callback.print = True

    #     prediction_callback.no_match_count = 0
    #     prediction_callback.iou_list = []

    if not hasattr(prediction_callback, "loop_counter"):
        prediction_callback.loop_counter = yolo_frame
        prediction_callback.rectangles = None
        prediction_callback.patches = None

    if not hasattr(prediction_callback, 'times'):
        prediction_callback.times = []
        prediction_callback.loops = -100
        prediction_callback.print = True

    global polygons_pub, mean_time_pub, std_time_pub

    start_time = time.time()

    bridge = CvBridge()
    try:
        # Convert the ROS Image message to a OpenCV image
        cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

        # send_time = data.header.stamp
        # receive_time = rospy.Time.now()
        # latency = round((receive_time - send_time).to_sec() * 1000, 3) 
        # prediction_callback.latency_counter = prediction_callback.latency_counter + 1
        # prediction_callback.latency_avg = prediction_callback.latency_avg + latency
        # print(f"\rlatency: {latency:.3f} ms  |  Avg = {prediction_callback.latency_avg/prediction_callback.latency_counter:.3f} ms     ", end="")
        
        # # Display the image using OpenCV
        # cv.imshow("Received Image", cv_image)
        # cv.waitKey(1)

        
        # ml_time = 0
        # pm_time = 0

        prediction_callback.rectangles, prediction_callback.patches = run_prediction(tensorrt_model, cv_image) # run_prediction(tensorrt_model, warm_up_image)

        # if prediction_callback.loop_counter >= yolo_frame:
        #     # Run yolo prediction and reset loop counter
        #     #prediction_callback.rectangles, prediction_callback.patches, ml_time = run_prediction(tensorrt_model, cv_image) # run_prediction(tensorrt_model, warm_up_image)
        #     prediction_callback.rectangles, prediction_callback.patches = run_prediction(tensorrt_model, cv_image) # run_prediction(tensorrt_model, warm_up_image)
        #     prediction_callback.loop_counter = 0
        # else:
        #     # Patch match 
        #     #prediction_callback.rectangles, prediction_callback.patches, pm_time = patch_match(cv_image, prediction_callback.patches, prediction_callback.rectangles)
        #     prediction_callback.rectangles, prediction_callback.patches = patch_match(cv_image, prediction_callback.patches, prediction_callback.rectangles)
            
        #     # Get YOLO bounding boxes for comparison
        #     # gt_rects, _notneeded1, _notneeded2 = run_prediction(tensorrt_model, cv_image) # run_prediction(tensorrt_model, warm_up_image)

        #     #matches, non_match_patch, non_match_ml = compare_boxes(gt_rects, prediction_callback.rectangles, 0.4)
        #     #prediction_callback.no_match_count += len(non_match_patch) + len(non_match_ml)
        #     #iou_vals = [mp[-1] for mp in matches]
        #     #prediction_callback.iou_list += iou_vals

        #     # Increase loop counter
        #     prediction_callback.loop_counter += 1

        
        
        # if prediction_callback.latency_counter < 0:
        #     prediction_callback.latency_counter += 1
        # elif prediction_callback.latency_counter >= 2000:
        #     if prediction_callback.print:
        #         print(f"PM mean time: {np.mean(prediction_callback.pm_times, dtype=np.float64):.2f}  |  std: {np.std(prediction_callback.pm_times, dtype=np.float64):.4f}")
        #         print(f"ML mean time: {np.mean(prediction_callback.ml_times, dtype=np.float64):.2f}  |  std: {np.std(prediction_callback.ml_times, dtype=np.float64):.4f}")
        #         print(f"Total mean time: {np.mean(prediction_callback.times, dtype=np.float64):.2f}  |  std: {np.std(prediction_callback.times, dtype=np.float64):.4f}")
        #         print(f"Mean iou: {np.mean(prediction_callback.iou_list, dtype=np.float64):.3f}  |  std: {np.std(prediction_callback.iou_list, dtype=np.float64):.4f}")
        #         #print(f"Average unmatched per frame: {(prediction_callback.no_match_count / (1000+20*(1+yolo_frame)) ):.3f}")
        #         #print(f"{prediction_callback.times[:30]}")
        #     prediction_callback.print = False
        # else:
        #     prediction_callback.latency_counter += 1
        #     if pm_time == 0:
        #         prediction_callback.ml_times.append(ml_time)
        #         prediction_callback.times.append(ml_time)
        #     elif ml_time == 0:
        #         prediction_callback.times.append(pm_time)
        #         prediction_callback.pm_times.append(pm_time)

        #     if prediction_callback.print:
        #         print(f"last 100 time: {np.sum(prediction_callback.times[max(-101, -1*len(prediction_callback.times)):-1]):.2f}")

        

            

        polygon_msg = Polygon()

        for rect in prediction_callback.rectangles:
            for corner in rect:
                point = Point32(x=corner[0], y=corner[1], z=0.0)
                polygon_msg.points.append(point)

        # print("sending rects")
        polygons_pub.publish(polygon_msg)


    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))
        print("error decoding image")
    
    end_time = time.time()
    elapse_time = (end_time - start_time) * 1000 # ms
    #avg_time = 0

    if prediction_callback.loops < 0:
        prediction_callback.loops += 1
    elif prediction_callback.loops > 5000:
        #avg_time = np.average(prediction_callback.times)
        if prediction_callback.print:
            mean_time = np.mean(prediction_callback.times, dtype=np.float64)
            std_time = np.std(prediction_callback.times, dtype=np.float64)
            mean_time_pub.publish(mean_time)
            std_time_pub.publish(std_time)
        prediction_callback.print = False
    else:
        prediction_callback.times.append(elapse_time)
        prediction_callback.loops += 1
        #avg_time = np.average(prediction_callback.times)  

    # if prediction_callback.print:
    #     print(f"Match time: {elapse_time:.2f}  |  {avg_time:.2f}")



def run_prediction(model, image):
    # if not hasattr(run_prediction, "loops"):
    #     run_prediction.times = []
    #     run_prediction.loops = -100
    #     run_prediction.print = True

    #start_time = time.time()

    results = model.predict(image, imgsz=(480,640), conf=0.4, iou=0.8, half=True, verbose=False)
    results = results[0] 
    regions = []
    patches = []
    for i in range(len(results.boxes)):
        box = results.boxes[i]
        tensor = box.xyxy[0]
        x1 = int(tensor[0].item())
        y1 = int(tensor[1].item())
        x2 = int(tensor[2].item())
        y2 = int(tensor[3].item())

        roi = image[y1:y2, x1:x2]
        patches.append(roi)

        #cv.rectangle(image, (x1,y1), (x2,y2), (255,0,255), 3)
        regions.append(((x1,y1),(x2,y2)))
    # cv.imshow("Image", image)
    # cv.waitKey(1)

    #end_time = time.time()
    #elapse_time = (end_time - start_time) * 1000 # ms
    #avg_time = 0

    # if run_prediction.loops < 0:
    #     run_prediction.loops += 1
    # elif run_prediction.loops > 2000:
    #     avg_time = np.average(run_prediction.times)
    #     if run_prediction.print:
    #         print(f"mean time: {np.mean(run_prediction.times, dtype=np.float64):.3f}  |  std: {np.std(run_prediction.times, dtype=np.float64):.4f}")
    #     run_prediction.print = False
    # else:
    #     run_prediction.times.append(elapse_time)
    #     run_prediction.loops += 1
    #     avg_time = np.average(run_prediction.times)  

    # if run_prediction.print:
    #     print(f"Match time: {elapse_time:.2f}  |  {avg_time:.2f}")



    # Optionally draw rectangles on the image for visualization
    # disp_copy = image.copy()
    # for reg in regions:
    #     cv.rectangle(disp_copy, reg[0], reg[1], (255, 0, 255), 3)
    # cv.imshow("YOLOv8", disp_copy)
    # cv.waitKey(1)

    # print(regions)

    return regions, patches #, elapse_time



# Match the image patch (from a previous frame) to its most similar location
def patch_match(image, patches, rects, search_window_size=10, scale=0.25):
    # if not hasattr(patch_match, "loops"):
    #     patch_match.times = []
    #     patch_match.loops = -100
    #     patch_match.print = True

    # start_time = time.time()
    updated_patches = []
    updated_rectangles = []

    image_small = cv.resize(image, None, fx=scale, fy=scale, interpolation=cv.INTER_LINEAR)

    
    for ind, patch in enumerate(patches):
        patch = cv.resize(patch, None, fx=scale, fy=scale, interpolation=cv.INTER_LINEAR)

        # Define the search window around the original rectangle
        search_x1 = max(0, int(rects[ind][0][0]*scale) - search_window_size)
        search_y1 = max(0, int(rects[ind][0][1]*scale) - search_window_size)
        search_x2 = min(image_small.shape[1], int(rects[ind][1][0]*scale) + search_window_size)
        search_y2 = min(image_small.shape[0], int(rects[ind][1][1]*scale) + search_window_size)
        search_region = image_small[search_y1:search_y2, search_x1:search_x2]

        # for scale in scales:
        #     # Resize the patch
        #     scaled_patch = cv.resize(patch, None, fx=scale, fy=scale, interpolation=cv.INTER_LINEAR)

        # Perform template matching
        result = cv.matchTemplate(search_region, patch, cv.TM_CCOEFF_NORMED)

        # Get the best match position for this scale
        _, max_val, _, max_loc = cv.minMaxLoc(result)

        best_patch = cv.resize(patch, None, fx=1/scale, fy=1/scale, interpolation=cv.INTER_LINEAR)
        # Calculate the new rectangle coordinates
        patch_h, patch_w = patch.shape[:2]
        new_x1 = int((search_x1 + max_loc[0]) / scale)
        new_y1 = int((search_y1 + max_loc[1]) / scale)
        new_x2 = int((search_x1 + max_loc[0] + patch_w) / scale)
        new_y2 = int((search_y1 + max_loc[1] + patch_h) / scale)
        best_rect = ((new_x1, new_y1), (new_x2, new_y2))

        # Append the updated patch and rectangle
        updated_patches.append(best_patch)
        updated_rectangles.append(best_rect)
    
    # end_time = time.time()
    # elapse_time = (end_time - start_time) * 1000 # ms
    # avg_time = 0

    # if patch_match.loops < 0:
    #     patch_match.loops += 1
    # elif patch_match.loops > 2000:
    #     avg_time = np.average(patch_match.times)
    #     if patch_match.print:
    #         print(f"mean time: {np.mean(patch_match.times, dtype=np.float64):.3f}  |  std: {np.std(patch_match.times, dtype=np.float64):.4f}")
    #     patch_match.print = False
    # else:
    #     patch_match.times.append(elapse_time)
    #     patch_match.loops += 1
    #     avg_time = np.average(patch_match.times)  

    # if patch_match.print:
    #     print(f"Match time: {elapse_time:.2f}  |  {avg_time:.2f}")


    # Optionally draw rectangles on the image for visualization
    # disp_copy = image.copy()
    # for region in updated_rectangles:
    #     cv.rectangle(disp_copy, region[0], region[1], (255, 0, 255), 3)
    # cv.imshow("Patch Match", disp_copy)
    # cv.waitKey(1)

    return updated_rectangles, updated_patches #, elapse_time


# # Function to calculate IoU between two boxes
# def calculate_iou(box1, box2):
#     p1, p2 = box1
#     p1_gt, p2_gt = box2

#     x1, y1 = p1
#     x2, y2 = p2
#     x1_gt, y1_gt = p1_gt
#     x2_gt, y2_gt = p2_gt

#     # Determine the coordinates of the intersection rectangle
#     x_left = max(x1, x1_gt)
#     y_top = max(y1, y1_gt)
#     x_right = min(x2, x2_gt)
#     y_bottom = min(y2, y2_gt)

#     if x_right < x_left or y_bottom < y_top:
#         return 0.0  # No overlap

#     # Area of the intersection rectangle
#     intersection_area = (x_right - x_left) * (y_bottom - y_top)

#     # Areas of the bounding boxes
#     box1_area = (x2 - x1) * (y2 - y1)
#     box2_area = (x2_gt - x1_gt) * (y2_gt - y1_gt)

#     # Area of the union
#     union_area = box1_area + box2_area - intersection_area

#     # IoU is intersection area over union area
#     return intersection_area / union_area

# # Function to compare two lists of boxes
# def compare_boxes(ml_boxes, template_boxes, iou_threshold=0.5):
#     matched_pairs = []
#     unmatched_ml_boxes = []
#     unmatched_template_boxes = []

#     # Create IoU matrix between ML boxes and template boxes
#     iou_matrix = np.zeros((len(template_boxes), len(ml_boxes)))

#     for i, t_box in enumerate(template_boxes):
#         for j, ml_box in enumerate(ml_boxes):
#             iou_matrix[i, j] = calculate_iou(t_box, ml_box)

#     # For each template box, find the ML box with the highest IoU
#     for i, t_box in enumerate(template_boxes):
#         best_ml_idx = np.argmax(iou_matrix[i])
#         best_iou = iou_matrix[i, best_ml_idx]

#         if best_iou >= iou_threshold:
#             matched_pairs.append((t_box, ml_boxes[best_ml_idx], best_iou))
#         else:
#             unmatched_template_boxes.append(t_box)

#     # Find unmatched ML boxes
#     matched_ml_indices = {pair[1] for pair in matched_pairs}
#     unmatched_ml_boxes = [box for box in ml_boxes if box not in matched_ml_indices]

#     return matched_pairs, unmatched_template_boxes, unmatched_ml_boxes




if __name__ == "__main__":
    main()