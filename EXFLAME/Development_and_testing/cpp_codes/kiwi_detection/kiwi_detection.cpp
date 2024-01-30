#include <opencv2/opencv.hpp>
#include <opencv2/cudawarping.hpp>
#include <opencv2/cudaimgproc.hpp>
//#include <opencv2/cudastereo.hpp>
#include <opencv2/cudafilters.hpp>
#include <pylon/PylonIncludes.h>
#include <iostream>
#include <chrono>
#include <tbb/parallel_for.h>

using namespace cv;
using namespace Pylon;

// Rounds a float to the nearest integer
int int_round(float val){
    return (int)(val + 0.5 - (val<0));
}

/* Takes the two rectification maps (returned by initUndistortRectifyMap), one points to the x location, and the other points to the y.
 * Computes a single map (lookup table), which points to the memory offset location of the pixel, instead of the x and y coordinates. 
 * Note: was created in order to use the function below (remap_colour) correctly. */
void precalc_map(const Mat& input_x, const Mat& input_y, std::vector<int>& output_map) {
    // Consistency check
    CV_Assert(input_x.size() == input_y.size());
    
    // Allocate output memory
    if (output_map.size() != (input_x.cols * input_x.rows)) {
        output_map.resize((input_x.cols * input_x.rows));
    }
    
    // Loop through the rectification map, calculating the memory offset of the pixel from the unrectified image
    int idx=0;
    for (int i=0; i<input_x.rows; i++) {
        for (int j=0; j<input_x.cols; j++) {
            int y_coord = int_round(input_y.at<float>(i,j));
            int x_coord = int_round(input_x.at<float>(i,j));
            if ((x_coord < 0) || (x_coord >= input_x.cols) || (y_coord < 0) || (y_coord >= input_x.rows)) {
                output_map[idx] = -1;
            }
            else {
                output_map[idx] = y_coord*input_x.cols*3 + x_coord*3;
            }
            // std::cout << "x: " << x_coord << " | y: " << y_coord << " | addr: " << output_map[idx] << std::endl;
            idx++;
        }
    }

    return;
}

/* Optimised remapping function which utilises a lookup table pointing to the pixel memory offset, which allows quicker computation.
 * Also optimised to copy 
 * Note: original code found at (https://trace.tennessee.edu/cgi/viewcontent.cgi?article=2196&context=utk_gradthes), minor modificaitons
 * have been made to allow it to work on linux. */
void remap_color(const Mat& input, Mat& output, std::vector<int>& map) {
    int chunksize = 24; // Equivalent to the number of threads created.
    // Note: the number of pixels in the image must be divisible by this number, else it will create colour shifted bands on the image. 

    // consistancy check
    CV_Assert(input.isContinuous());
    CV_Assert((input.cols * input.rows) == map.size());

    // allocate output Mat memmory
    if (input.size() != output.size()) {
        output.release();
        output.create(input.size(), input.type());
    }

    int chunk_width = map.size() * 3 / chunksize;
    int map_width = map.size() / chunksize;
    int max_addr_offset = map_width -1;

    tbb::parallel_for(0, chunksize, [&](int chunk) {
        int m;
        int *mapptr = &(map[0]) + map_width * chunk;
        uchar *data = output.data + chunk_width * chunk;
        int *indata;
        int *maxaddess = mapptr + max_addr_offset;   
        for (; mapptr < maxaddess; mapptr++) {
            // Copies a 4-byte section of data (3 bytes for R,G,B, and 1 dummy byte)
            m = mapptr[0];
            if (m < 0) {
                *((int*)data) = 0;
            }
            else {
                indata = (int*)&(input.data[m]);
                *((int*)data) = *indata;
            }
            // Increments pointer by 3 bytes, to cause the dummy byte to be overwritten on the next loop
            data += 3;
        }
        // Copies the last 3 bytes individually, to avoid changing memory beyond what it should
        m = *mapptr;
        if (m < 0) {
            data[0] = 0;
            data[1] = 0;
            data[2] = 0;
        }
        else {
            indata = (int*)&(input.data[m]);
            data[0] = ((uchar*)indata)[0];
            data[1] = ((uchar*)indata)[1];
            data[2] = ((uchar*)indata)[2];
        }
    });

    return;
}

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
    get_rectifier_maps(0, map_left_x, map_left_y, map_right_x, map_right_y, roi_left, roi_right);

    std::vector<int> map_left, map_right;
    precalc_map(map_left_x, map_left_y, map_left);
    precalc_map(map_right_x, map_right_y, map_right);

    // const int num_disp = 64;
    // const int block_size = 17;

    // Create a stereo block matching object
    // Ptr<StereoSGBM> stereo = StereoSGBM::create(
    //     1, num_disp, block_size,
    //     3 * 3 * block_size * block_size,
    //     5 * 3 * block_size * block_size,
    //     0, 0, 200, 2
    // );

    const int focal_length_pixel = 855;
    const double baseline = 0.03316; // meters
    const double baseline_mm = baseline * 1000; // mm

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

        auto total_duration = std::chrono::microseconds(0);
        auto kiwi_duration = std::chrono::microseconds(0);
        auto custom_duration = std::chrono::microseconds(0);
        auto morph_duration = std::chrono::microseconds(0);

        CGrabResultPtr grabResultRight;
        CGrabResultPtr grabResultLeft;

        cuda::GpuMat leftimg(480, 640, CV_8UC1);
        cuda::GpuMat rightimg(480, 640, CV_8UC1);

        cuda::GpuMat map_x_left;
        cuda::GpuMat map_y_left;
        map_x_left.upload(map_left_x);
        map_y_left.upload(map_left_y);

        cuda::GpuMat map_x_right;
        cuda::GpuMat map_y_right;
        map_x_right.upload(map_right_x);
        map_y_right.upload(map_right_y);

        cuda::GpuMat color_leftimg(480, 640, CV_8UC3);
        cuda::GpuMat color_rightimg(480, 640, CV_8UC3);

        cuda::GpuMat rect_leftimg(480, 640, CV_8UC3);
        cuda::GpuMat rect_rightimg(480, 640, CV_8UC3);

        Mat leftImage(480, 640, CV_8UC1);
        Mat rightImage(480, 640, CV_8UC1);

        Mat color_left_image(480, 640, CV_8UC3);
        Mat color_right_image(480, 640, CV_8UC3);
        Mat rectified_left(480, 640, CV_8UC3);
        Mat rectified_right(480, 640, CV_8UC3);
        
        Mat hsvImage(480, 640, CV_8UC3);
        Mat yellowMask;
        Mat kernel_c, kernel_o;
        Mat closing;
        Mat opening;
        std::vector<std::vector<Point>> contours;

        Mat roi_img;
        Mat template_img;

        Mat rect_r_copy(480, 640, CV_8UC3);
        Mat match_result;

        double min_val, max_val;
        Point min_loc, max_loc;
        int disparity, depth_mm;

        int hue_lower=15, hue_upper=38, sat_lower=130, sat_upper=255, val_lower=20, val_upper=230;
        int radiusClose=0, radiusOpen=1; // radiusClose=2, radiusOpen=6;
        int minAreaThreshold=350; //, maxAreaThreshold=50000;
        // namedWindow("Track Bar Window", WINDOW_NORMAL);
        // createTrackbar("Hue Lower", "Track Bar Window", &hue_lower, 180, NULL, NULL);
        // createTrackbar("Hue Upper", "Track Bar Window", &hue_upper, 180, NULL, NULL);
        // createTrackbar("Saturation Lower", "Track Bar Window", &sat_lower, 255, NULL, NULL);
        // createTrackbar("Saturation Higher", "Track Bar Window", &sat_upper, 255, NULL, NULL);
        // createTrackbar("Value Lower", "Track Bar Window", &val_lower, 255, NULL, NULL);
        // createTrackbar("Value Upper", "Track Bar Window", &val_upper, 255, NULL, NULL);
        // createTrackbar("Radius Close", "Track Bar Window", &radiusClose, 15, NULL, NULL);
        // createTrackbar("Radius Open", "Track Bar Window", &radiusOpen, 15, NULL, NULL);
        // createTrackbar("Min Area", "Track Bar Window", &minAreaThreshold, 5000, NULL, NULL);
        // createTrackbar("Max Area", "Track Bar Window", &maxAreaThreshold, 100000, NULL, NULL);

        // const Scalar lowerYellow(hue_lower, sat_lower, val_lower);
        // const Scalar upperYellow(hue_upper, sat_upper, val_upper);

        // kernel_c = getStructuringElement(MORPH_ELLIPSE, Size(2*radiusClose+1, 2*radiusClose+1));
        // kernel_o = getStructuringElement(MORPH_ELLIPSE, Size(2*radiusOpen+1, 2*radiusOpen+1));

        int loops = 0;
        int custom_loops = 0;
        int key = 0;
        while (camera_left.IsGrabbing() && camera_right.IsGrabbing() && key != 27) {
            
            loops++;
            
            auto start = std::chrono::high_resolution_clock::now();

            // Grab and retrieve images from camera_left
            camera_left.RetrieveResult(500, grabResultLeft, TimeoutHandling_ThrowException);
            if (grabResultLeft->GrabSucceeded()) {
                leftImage.data = static_cast<uchar*>(grabResultLeft->GetBuffer());
            }
            
            // Grab and retrieve images from camera_right
            camera_right.RetrieveResult(500, grabResultRight, TimeoutHandling_ThrowException);
            if (grabResultRight->GrabSucceeded()) {
                rightImage.data = static_cast<uchar*>(grabResultRight->GetBuffer());
            }

            cvtColor(leftImage, color_left_image, COLOR_BayerBG2RGB);
            cvtColor(rightImage, color_right_image, COLOR_BayerBG2RGB);
            

            remap_color(color_left_image, rectified_left, map_left);
            remap_color(color_right_image, rectified_right, map_right);


            // leftimg.upload(color_left_image);
            // rightimg.upload(color_right_image);
             
            // cuda::remap(leftimg, rect_leftimg, map_x_left, map_y_left, INTER_LINEAR);
            // cuda::remap(rightimg, rect_rightimg, map_x_right, map_y_right, INTER_LINEAR);            
            
            // rect_leftimg.download(rectified_left);
            // rect_rightimg.download(rectified_right);

            

            // rectangle(rectified_right, roi_right, Scalar(0, 255, 0), 1);
            // rectangle(rectified_left, roi_left, Scalar(0, 255, 0), 1);


            auto start_kiwi = std::chrono::high_resolution_clock::now();
            
            Scalar lowerYellow(hue_lower, sat_lower, val_lower);
            Scalar upperYellow(hue_upper, sat_upper, val_upper);
            
            rectified_right.copyTo(rect_r_copy);
            
            cvtColor(rectified_right, hsvImage, COLOR_BGR2HSV);
            inRange(hsvImage, lowerYellow, upperYellow, yellowMask);
             
            kernel_c = getStructuringElement(MORPH_ELLIPSE, Size(2*radiusClose+1, 2*radiusClose+1));
            kernel_o = getStructuringElement(MORPH_ELLIPSE, Size(2*radiusOpen+1, 2*radiusOpen+1));
            
            auto start_morph = std::chrono::high_resolution_clock::now(); 
            // opening = yellowMask;
            morphologyEx(yellowMask, closing, MORPH_CLOSE, kernel_c);
            morphologyEx(closing, opening, MORPH_OPEN, kernel_o);
            auto end_morph = std::chrono::high_resolution_clock::now();

            findContours(opening, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
            for (const auto& contour : contours) {
                double area = contourArea(contour);
                if (area > minAreaThreshold) { //&& area < maxAreaThreshold
                    custom_loops++;
                    auto start_custom = std::chrono::high_resolution_clock::now(); // CUSTOM START ##############################################################
                    Rect boundingBox = boundingRect(contour);
                    rectangle(rect_r_copy, boundingBox, Scalar(0, 255, 0), 2); 
                    Point circ_centre(boundingBox.x+boundingBox.width/2, boundingBox.y+boundingBox.height/2);
                    circle(rect_r_copy, circ_centre, 5, Scalar(255,0,0), FILLED);

                    // Creates an roi to force horizontal sliding between 10 to 50 pixels (right)
                    int x_start = boundingBox.x+10;
                    int x_width = boundingBox.width+150;
                    if (x_start + x_width >= rectified_left.cols) continue;

                    //Rect roi_match(x_start, boundingBox.y, x_width, boundingBox.height);
                    Rect roi_match(x_start, circ_centre.y, x_width, 1);
                    Rect roi_template(boundingBox.x, circ_centre.y, boundingBox.width, 1);
                    roi_img = rectified_left(roi_match);
                    template_img = rectified_right(roi_template);
                    
                    // Perform template matching
                    matchTemplate(roi_img, template_img, match_result, TM_SQDIFF_NORMED);
                    minMaxLoc(match_result, &min_val, &max_val, &min_loc, &max_loc);
                    
                    circle(rectified_left, Point(circ_centre.x+min_loc.x+10,circ_centre.y), 5, Scalar(255,0,0), FILLED);

                    disparity = 10 + min_loc.x;
                    depth_mm = focal_length_pixel * baseline_mm / disparity;

                    double xmm, ymm, zmm=depth_mm;
                    xmm = zmm * (circ_centre.x - 640) / 854;
                    ymm = zmm * (480 - circ_centre.y) / 856;
                    double distmm = sqrt((pow(xmm,2) + pow(ymm,2) + pow(zmm,2)));
                    
                    putText(rect_r_copy, std::to_string(distmm), Point(boundingBox.x,boundingBox.y), FONT_HERSHEY_PLAIN, 2, Scalar(255,0,0), 2);
                    auto end_custom = std::chrono::high_resolution_clock::now(); // CUSTOM END ################################################################### 
                    auto duration_custom = std::chrono::duration_cast<std::chrono::microseconds>(end_custom - start_custom);
                    custom_duration += duration_custom;
                }
            }
            
            auto end_kiwi = std::chrono::high_resolution_clock::now();
            auto duration_kiwi = std::chrono::duration_cast<std::chrono::microseconds>(end_kiwi - start_kiwi);
            kiwi_duration += duration_kiwi;

            auto end = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
            total_duration += duration;

             
            auto duration_morph = std::chrono::duration_cast<std::chrono::microseconds>(end_morph - start_morph);
            morph_duration += duration_morph;

            
            // imshow("Base", roi_img);
            // imshow("Template", template_img);

            imshow("Kiwi mask", opening);
            imshow("Kiwi distances", rect_r_copy);

            // cv::imshow("Result", color_left_image);
            // cv::waitKey(0);

            // Display the rectified images
            // imshow("Left", color_left_image);
            // imshow("Right", color_right_image);
            imshow("Left rectified", rectified_left);
            imshow("Right rectified", rectified_right);

            key = waitKey(1);
        }

        auto average_duration = total_duration / loops;
        std::cout << "Rectify time: " << average_duration.count() << " us" << std::endl;
        auto average_kiwi = kiwi_duration / loops;
        std::cout << "Kiwi time: " << average_kiwi.count() << " us" << std::endl;
        auto average_morph = morph_duration / loops;
        std::cout << "Morph time: " << average_morph.count() << " us" << std::endl;
        auto average_custom = custom_duration / custom_loops;
        std::cout << "Custom timer: " << average_custom.count() << " us" << std::endl;

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