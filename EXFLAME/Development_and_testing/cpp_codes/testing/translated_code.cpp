#include <opencv2/opencv.hpp>
#include <pylon/PylonIncludes.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

using namespace cv;
using namespace Pylon;

void get_rectifier_maps(double a, Mat& map_left_x, Mat& map_left_y, Mat& map_right_x, Mat& map_right_y, Rect& roi_left, Rect& roi_right) {
    // Create camera parameters
    Mat intrinsics_left = (Mat_<double>(3, 3) <<
        854.189844354725, -0.247393906478783, 333.025264031272,
        0, 856.02238812754, 239.36139034707,
        0, 0, 1);
    Mat distortion_left = (Mat_<double>(1, 5) <<
        -0.245533070243884, 0.118136759247279, 0.00110572988534011, 0.000277690748779616, 0);
    Mat intrinsics_right = (Mat_<double>(3, 3) <<
        854.437943123657, -0.186007786296275, 330.918630043833,
        0, 856.27619444407, 245.050063439282,
        0, 0, 1);
    Mat distortion_right = (Mat_<double>(1, 5) <<
        -0.251983366957398, 0.142729877568207, 0.00110572988534011, 0.000277690748779616, 0);
    Mat extrinsics_R = (Mat_<double>(3, 3) <<
        0.999901831378116, 0.00836990908895404, 0.0112370916402682,
        -0.00830392579649652, 0.999948081870091, -0.00590579213076292,
        -0.0112859391747174, 0.00581190039213907, 0.999919421448937);
    Mat extrinsics_T = (Mat_<double>(3, 1) <<
        -33.1599491546372, -0.0683590933414245, -0.292426939357079);

    // Create stereo rectification maps
    Mat R1, R2, P1, P2, Q;
    stereoRectify(
        intrinsics_left, distortion_left,
        intrinsics_right, distortion_right,
        Size(640, 480), // specify the size of your images
        extrinsics_R, extrinsics_T,
        R1, R2, P1, P2, Q,
        CALIB_ZERO_DISPARITY, a, Size(640, 480), &roi_left, &roi_right
    );
    // Compute the rectification maps
    initUndistortRectifyMap(
        intrinsics_left, distortion_left, R1, P1,
        Size(640, 480), CV_32FC1, map_left_x, map_left_y
    );
    initUndistortRectifyMap(
        intrinsics_right, distortion_right, R2, P2,
        Size(640, 480), CV_32FC1, map_right_x, map_right_y
    );
}

int main() {
    Mat map_left_x, map_left_y, map_right_x, map_right_y;
    Rect roi_left, roi_right;
    // get_rectifier_maps(1, map_left_x, map_left_y, map_right_x, map_right_y, roi_left, roi_right);

    Mat intrinsics_left = (Mat_<double>(3, 3) <<
        854.189844354725, -0.247393906478783, 333.025264031272,
        0, 856.02238812754, 239.36139034707,
        0, 0, 1);
    Mat distortion_left = (Mat_<double>(1, 5) <<
        -0.245533070243884, 0.118136759247279, 0.00110572988534011, 0.000277690748779616, 0);
    Mat intrinsics_right = (Mat_<double>(3, 3) <<
        854.437943123657, -0.186007786296275, 330.918630043833,
        0, 856.27619444407, 245.050063439282,
        0, 0, 1);
    Mat distortion_right = (Mat_<double>(1, 5) <<
        -0.251983366957398, 0.142729877568207, 0.00110572988534011, 0.000277690748779616, 0);
    Mat extrinsics_R = (Mat_<double>(3, 3) <<
        0.999901831378116, 0.00836990908895404, 0.0112370916402682,
        -0.00830392579649652, 0.999948081870091, -0.00590579213076292,
        -0.0112859391747174, 0.00581190039213907, 0.999919421448937);
    Mat extrinsics_T = (Mat_<double>(3, 1) <<
        -33.1599491546372, -0.0683590933414245, -0.292426939357079);

    // Create stereo rectification maps
    Mat R1, R2, P1, P2, Q;
    stereoRectify(
        intrinsics_left, distortion_left,
        intrinsics_right, distortion_right,
        Size(640, 480), // specify the size of your images
        extrinsics_R, extrinsics_T,
        R1, R2, P1, P2, Q,
        CALIB_ZERO_DISPARITY, 1, Size(640, 480), &roi_left, &roi_right
    );
    // Compute the rectification maps
    initUndistortRectifyMap(
        intrinsics_left, distortion_left, R1, P1,
        Size(640, 480), CV_32FC1, map_left_x, map_left_y
    );
    initUndistortRectifyMap(
        intrinsics_right, distortion_right, R2, P2,
        Size(640, 480), CV_32FC1, map_right_x, map_right_y
    );

    int num_disp = 64;
    int block_size = 17;

    // Create a stereo block matching object
    Ptr<StereoSGBM> stereo = StereoSGBM::create(
        1, num_disp, block_size,
        3 * 3 * block_size * block_size,
        5 * 3 * block_size * block_size,
        0, 0, 200, 2
    );

    int focal_length_pixel = 855;
    double baseline = 0.03316; // meters
    double baseline_mm = baseline * 1000; // mm

    PylonInitialize();

    // Get all available devices
    CTlFactory& tlFactory = CTlFactory::GetInstance();
    DeviceInfoList_t devices;
    tlFactory.EnumerateDevices(devices);

    if (devices.size() < 2) {
        std::cout << "Less than 2 cameras found." << std::endl;
        return -1;
    }

    // Create InstantCamera objects for left and right cameras
    CInstantCamera camera_left(tlFactory.CreateDevice(devices[0])); // SN: 24522821 on left
    CInstantCamera camera_right(tlFactory.CreateDevice(devices[1])); // SN: 24810503 on right
    // Note: devices are always ordered with the lowest Serial Number (SN) first.

    try {
        // Open cameras
        camera_left.Open();
        camera_right.Open();

        // Start grabbing images for left camera
        camera_left.StartGrabbing(GrabStrategy_LatestImageOnly);
        // Start grabbing images for right camera
        camera_right.StartGrabbing(GrabStrategy_LatestImageOnly);

        int key = 0;
        while (camera_left.IsGrabbing() && camera_right.IsGrabbing() && key != 27) {
            // Grab and retrieve images from camera_left
            CGrabResultPtr grabResultLeft;
            camera_left.RetrieveResult(5000, grabResultLeft, TimeoutHandling_ThrowException);

            if (grabResultLeft->GrabSucceeded()) {
                Mat leftImage(grabResultLeft->GetHeight(), grabResultLeft->GetWidth(), CV_8UC1,
                                grabResultLeft->GetBuffer());

                // Grab and retrieve images from camera_right
                CGrabResultPtr grabResultRight;
                camera_right.RetrieveResult(5000, grabResultRight, TimeoutHandling_ThrowException);

                if (grabResultRight->GrabSucceeded()) {
                    Mat rightImage(grabResultRight->GetHeight(), grabResultRight->GetWidth(), CV_8UC1,
                                    grabResultRight->GetBuffer());

                    Mat color_left_image;
                    cvtColor(leftImage, color_left_image, COLOR_BayerBG2RGB);
                    
                    Mat color_right_image;
                    cvtColor(rightImage, color_right_image, COLOR_BayerBG2RGB);
                    
                    Mat rectified_right;
                    remap(color_right_image, rectified_right, map_right_x, map_right_y, INTER_LINEAR);

                    Mat rectified_left;
                    remap(color_left_image, rectified_left, map_left_x, map_left_y, INTER_LINEAR);

                    rectangle(rectified_right, roi_right, Scalar(0, 255, 0), 1);
                    rectangle(rectified_left,roi_left, Scalar(0, 255, 0), 1);
                    
                    // Display the rectified images
                    imshow("Right rectified", rectified_right);
                    imshow("Left rectified", rectified_left);
                    imshow("Right", color_right_image);
                    imshow("Left", color_left_image);

                    key = cv::waitKey(1);
                }
            }
        }

    } catch (const GenericException &e) {
        std::cerr << "An exception occurred: " << e.GetDescription() << std::endl;
        return -1;
    }
    
    // Stop grabbing images for both cameras
    camera_left.StopGrabbing();
    camera_right.StopGrabbing();

    // Close cameras
    camera_left.Close();
    camera_right.Close();

    // Cleanup
    PylonTerminate();
    std::cout << "Ending" << std::endl;

    return 0;
}