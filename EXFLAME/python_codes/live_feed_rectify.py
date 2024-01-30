import cv2 as cv
import numpy as np
from time import perf_counter
from pypylon import pylon

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
    # intrinsics_left = np.array([
    #     [854.189844354725, -0.247393906478783, 333.025264031272],
    #     [0, 856.02238812754, 239.36139034707],
    #     [0, 0, 1]])
    # distortion_left = np.array([
    #     -0.245533070243884, 0.118136759247279, 0.00110572988534011, 0.000277690748779616, 0])
    # intrinsics_right = np.array([
    #     [854.437943123657, -0.186007786296275, 330.918630043833],
    #     [0, 856.27619444407, 245.050063439282],
    #     [0, 0, 1]])
    # distortion_right = np.array([
    #     -0.251983366957398, 0.142729877568207, 0.00110572988534011, 0.000277690748779616, 0])
    # extrinsics_R = np.array([
    #     [0.999901831378116, 0.00836990908895404, 0.0112370916402682],
    #     [-0.00830392579649652, 0.999948081870091, -0.00590579213076292],
    #     [-0.0112859391747174, 0.00581190039213907, 0.999919421448937]])
    # extrinsics_T = np.array([[-33.1599491546372], [-0.0683590933414245], [-0.292426939357079]])

    # print(f"K_left:\n{intrinsics_left}\n\nK_right:\n{intrinsics_right}\n")
    # print(f"dist_left:\n{distortion_left}\n\ndist_right:\n{distortion_right}\n")
    # print(f"R:\n{extrinsics_R}\n\nT:\n{extrinsics_T}\n")

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
    
    return map_left_x, map_left_y, map_right_x, map_right_y, roi_left, roi_right



def main():
    map_left_x, map_left_y, map_right_x, map_right_y, roi_left, roi_right = get_rectifier_maps(1)

    num_disp = 64
    block_size = 17

    # Create a stereo block matching object
    stereo = cv.StereoSGBM_create(
        minDisparity=1, 
        numDisparities=num_disp, 
        blockSize=block_size,
        P1=3 * 3 * block_size ** 2,
        P2=5 * 3 * block_size ** 2,
        #preFilterCap=63,          # Adjust this parameter
        #uniquenessRatio=10,       # Adjust this parameter
        speckleWindowSize=200,    # Adjust this parameter
        speckleRange=2 
        )

    focal_length_pixel = 855
    baseline = 0.03316 # meters
    baseline_mm = baseline*1000 # mm

    # loops = 0
    # while True:
    #     if loops == 0:
    #         t1_start = perf_counter()
    #     loops += 1
    #     if loops > 100:
    #         t1_stop = perf_counter()
    #         t1_elapsed = ((t1_stop - t1_start) * 1000) / loops # time in ms
    #         print(f"Time: {t1_elapsed:>5.2f}ms", end="\r")
    #         loops = 0
    

    src = cv.cuda_GpuMat()
    mapx = cv.cuda_GpuMat()
    mapy = cv.cuda_GpuMat()
    mapx.upload(map_left_x)
    mapy.upload(map_left_y)



    tlf = pylon.TlFactory.GetInstance()

    # Get all available devices
    devices = tlf.EnumerateDevices()

    if len(devices) < 2:
        print("Less than 2 cameras found.")
        return
    
    # Maybe try this way??? has better scynchronisation
    # cam_array = pylon.InstantCameraArray(2)
    #
    # for idx, cam in enumerate(cam_array):
    #     cam.Attach(tlf.CreateDevice(devices[idx]))
    #
    #     if devices[idx].GetSerialNumber() == "24522821": # Left camera
    #         cam.SetCameraContext(4)
    #         #print("left set", devices[idx].GetSerialNumber())
    #     elif devices[idx].GetSerialNumber() == "24810503": # Right camera
    #         cam.SetCameraContext(5)
    #         #print("right set", devices[idx].GetSerialNumber())
    #     # else:
    #     #     print("i dont know the type")
    #     #     print(type(devices[idx].GetSerialNumber()))


    # Turns out the lower SN is always the first device
    # for idx, device in enumerate(devices):
    #     print(f"Device {idx} | SN: {device.GetSerialNumber()}")
    #     if device.GetSerialNumber() == "24522821": # Left camera
    #         camera_left = pylon.InstantCamera(tlf.CreateDevice(device))
    #         #print("left set", devices[idx].GetSerialNumber())
    #     elif device.GetSerialNumber() == "24810503": # Right camera
    #         camera_right = pylon.InstantCamera(tlf.CreateDevice(device))
    #         #print("right set", devices[idx].GetSerialNumber())

    # Therefore this will always be correct (so long as the cameras arent physically swapped in the holder)
    camera_left = pylon.InstantCamera(tlf.CreateDevice(devices[0]))
    camera_right = pylon.InstantCamera(tlf.CreateDevice(devices[1]))

    camera_left.Open()
    camera_right.Open()

    try:
        # Set camera parameters for camera1 if needed
        # camera1.ExposureTime.SetValue(10000)

        # Set camera parameters for camera2 if needed
        # camera2.ExposureTime.SetValue(10000)

        # Start grabbing images for camera_left
        camera_left.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)

        # Start grabbing images for camera2
        camera_right.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)

        gpu_time = 0
        cpu_time = 0
        loops = 0
        grab_time = 0

        #counter = 1
        key = 0
        while camera_left.IsGrabbing() and camera_right.IsGrabbing() and key != 27:

            loops += 1


            timagegrab_start = perf_counter()

            grab_result_left = camera_left.RetrieveResult(100)
            if grab_result_left.GrabSucceeded():
                left_image = grab_result_left.Array
            grab_result_left.Release()

            grab_result_right = camera_right.RetrieveResult(100)
            if grab_result_right.GrabSucceeded():
                right_image = grab_result_right.Array
            grab_result_right.Release()

            timagegrab_stop = perf_counter()


            timagegrab_elapsed = ((timagegrab_stop - timagegrab_start) * 1000)
            grab_time += timagegrab_elapsed
            
            
            # disparity_map = stereo.compute(rectified_left, rectified_right).astype(np.float32)/16
            # normalized_disparity_map = cv.normalize(disparity_map, None, 0, 255, cv.NORM_MINMAX).astype(np.uint8)


            tgpu_start = perf_counter()
            src.upload(left_image)
            col_conv = cv.cuda.cvtColor(src, cv.COLOR_BAYER_BG2RGB)
            dst = cv.cuda.remap(col_conv, mapx, mapy, cv.INTER_LINEAR)
            rectified_left = dst.download()
            tgpu_stop = perf_counter()

            tgpu_elapsed = ((tgpu_stop - tgpu_start) * 1000)
            gpu_time += tgpu_elapsed
            

            
            color_left_image = cv.cvtColor(left_image, cv.COLOR_BAYER_BG2RGB)

            tcpu_start = perf_counter()
            color_right_image = cv.cvtColor(right_image, cv.COLOR_BAYER_BG2RGB)
            rectified_right = cv.remap(color_right_image, map_right_x, map_right_y, cv.INTER_LINEAR)
            tcpu_stop = perf_counter()

            tcpu_elapsed = ((tcpu_stop - tcpu_start) * 1000)
            cpu_time += tcpu_elapsed

            x,y,w,h = roi_right
            cv.rectangle(rectified_right,(x,y),(x+w,y+h),(0,255,0),1)
            x,y,w,h = roi_left
            cv.rectangle(rectified_left,(x,y),(x+w,y+h),(0,255,0),1)
            

            # Display rectified images side by side
            cv.imshow("Left Rectified - GPU", rectified_left)
            cv.imshow('Right Rectified - CPU', rectified_right)

            
            # Display the images and disparity map
            cv.imshow("Left colour", color_left_image)
            cv.imshow("Right colour", color_right_image)
            # cv.imshow("Left",left_image)
            # cv.imshow("Right",right_image)
            # cv.imshow('Left rect', rectified_left)
            # cv.imshow('Disparity Map', normalized_disparity_map)
            key = cv.waitKey(1)
            
            # if key == 115:
            #     print("saving")
            #     cv.imwrite(f"left_image_{counter}.png",left_image)
            #     cv.imwrite(f"right_image_{counter}.png",right_image)
            #     counter += 1

        print(f"Grab Time: {grab_time/loops:>5.2f}ms\nCPU Time: {cpu_time/loops:>5.2f}ms\nGPU Time: {gpu_time/loops:>5.2f}ms\n")

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