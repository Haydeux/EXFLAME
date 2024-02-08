
// 265 Hz with no image display


#include <opencv2/opencv.hpp>
#include <opencv2/cudawarping.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudafilters.hpp>
#include <pylon/PylonIncludes.h>
#include <iostream>
#include <csignal>
#include <cstdlib>
#include <chrono>
#include <sstream>
#include <tbb/parallel_for.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PointStamped.h>

using namespace cv;
using namespace Pylon;



class Baslers {
    private:
        volatile bool* alive = nullptr;
        
    public: 
        // Constructor
        Baslers(volatile void* aliveRunning) { 
            alive = (volatile bool*)aliveRunning;
        }
        
        // Destructor
        //~Baslers() { }

        int main_program() {
            // Get the rectification maps
            Mat map_left_x, map_left_y, map_right_x, map_right_y;
            Rect roi_left, roi_right;
            get_rectifier_maps(0, map_left_x, map_left_y, map_right_x, map_right_y, roi_left, roi_right);

            // Convert the rectifcation maps pairs to a single memory offset lookup table
            std::vector<int> map_left, map_right;
            precalc_map(map_left_x, map_left_y, map_left);
            precalc_map(map_right_x, map_right_y, map_right);

            // Setup constant camera parameters
            const double focal_length_pixel = (854.189844 + 856.022388) / 2.0;
            const double baseline = 0.03316; // meters
            const double baseline_mm = baseline * 1000.0; // mm

            // Initialise the ros node and message
            int argc = 0;
            ros::init(argc, nullptr, "kiwi_sender");
            ros::NodeHandle kiwi_n;
            ros::Publisher kiwi_pub = kiwi_n.advertise<geometry_msgs::PointStamped>("FLAME/KiwiPos", 1);
            geometry_msgs::PointStamped kiwiPosMsg;

            // Initialise the camera software
            PylonInitialize();

            // Get all available camera devices
            CTlFactory& tlFactory = CTlFactory::GetInstance();
            DeviceInfoList_t devices;
            tlFactory.EnumerateDevices(devices);

            // Check to ensure there are 2 cameras
            if (devices.size() < 2) {
                std::cout << "Less than 2 cameras found." << std::endl;
                return -1;
            }
            else if (devices.size() > 2) {
                std::cout << "More than 2 cameras found." << std::endl;
                return -1;
            }

            // Create InstantCamera objects for left and right cameras
            CInstantCamera camera_left(tlFactory.CreateDevice(devices[0])); // SN: 24522821 on left
            CInstantCamera camera_right(tlFactory.CreateDevice(devices[1])); // SN: 24810503 on right
            // Note: devices are always ordered with the lowest Serial Number (SN) first.

            // Initialise some timers
            auto total_duration = std::chrono::microseconds(0);

            const int hue_lower=15, hue_upper=38, sat_lower=130, sat_upper=255, val_lower=20, val_upper=230;
            const int radiusClose=0, radiusOpen=1; 
            const int minAreaThreshold=350; 

            const Scalar lowerYellow(hue_lower, sat_lower, val_lower);
            const Scalar upperYellow(hue_upper, sat_upper, val_upper);

            const Mat kernel_c = getStructuringElement(MORPH_ELLIPSE, Size(2*radiusClose+1, 2*radiusClose+1));
            const Mat kernel_o = getStructuringElement(MORPH_ELLIPSE, Size(2*radiusOpen+1, 2*radiusOpen+1));

            CGrabResultPtr grabResultRight, grabResultLeft;

            Mat leftImage(480, 640, CV_8UC1);
            Mat rightImage(480, 640, CV_8UC1);

            Mat color_left_image(480, 640, CV_8UC3), color_right_image(480, 640, CV_8UC3);
            Mat rectified_left(480, 640, CV_8UC3), rectified_right(480, 640, CV_8UC3);
            
            Mat hsvImage(480, 640, CV_8UC3), yellowMask;
            Mat closing, opening;
            std::vector<std::vector<Point>> contours;

            Mat roi_img, template_img, roi_resized, template_resized;

            Mat rect_l_copy(480, 640, CV_8UC3);
            Mat match_result;

            double min_val, max_val;
            Point min_loc, max_loc;
            double disparity, depth_mm;

            std::cout << "Program starting" << std::endl;

            try {
                // Open cameras
                camera_left.Open();
                camera_right.Open();

                // Start grabbing images for the cameras
                camera_left.StartGrabbing(GrabStrategy_LatestImageOnly);
                camera_right.StartGrabbing(GrabStrategy_LatestImageOnly);
            
                int key = 0;
                unsigned long loops = 0; //, custom_loops = 0;
                while (camera_left.IsGrabbing() && camera_right.IsGrabbing() && key != 27 && *alive && ros::ok()) {
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

                    kiwiPosMsg.header.stamp = ros::Time::now();

                    cvtColor(leftImage, color_left_image, COLOR_BayerBG2RGB);
                    cvtColor(rightImage, color_right_image, COLOR_BayerBG2RGB);

                    remap_color(color_left_image, rectified_left, map_left);
                    remap_color(color_right_image, rectified_right, map_right);

                    // auto start_kiwi = std::chrono::high_resolution_clock::now();
                    
                    rectified_left.copyTo(rect_l_copy);
            
                    cvtColor(rectified_left, hsvImage, COLOR_BGR2HSV);
                    inRange(hsvImage, lowerYellow, upperYellow, yellowMask);
                    
                    // auto start_morph = std::chrono::high_resolution_clock::now(); 
                    morphologyEx(yellowMask, closing, MORPH_CLOSE, kernel_c);
                    morphologyEx(closing, opening, MORPH_OPEN, kernel_o);
                    // auto end_morph = std::chrono::high_resolution_clock::now();

                    findContours(opening, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
                    for (const auto& contour : contours) {
                        double area = contourArea(contour);
                        if (area > minAreaThreshold) { 
                            // custom_loops++;
                            // auto start_custom = std::chrono::high_resolution_clock::now(); 
                            Rect boundingBox = boundingRect(contour);
                            Point circ_centre(boundingBox.x+boundingBox.width/2, boundingBox.y+boundingBox.height/2);

                            // Display stuff
                            rectangle(rect_l_copy, boundingBox, Scalar(0, 255, 0), 2); 
                            circle(rect_l_copy, circ_centre, 5, Scalar(255,0,0), FILLED);

                            // Creates an roi to for horizontal sliding 
                            int x_start = boundingBox.x - 160;
                            int x_width = boundingBox.width + 150;
                            if (x_start + x_width >= rectified_left.cols) continue;
                            if (x_start < 0) continue;

                            Rect roi_match(x_start, circ_centre.y, x_width, 1); 
                            Rect roi_template(boundingBox.x, circ_centre.y, boundingBox.width, 1); 
                            roi_img = rectified_right(roi_match);
                            template_img = rectified_left(roi_template);

                            resize(roi_img, roi_resized, Size(roi_img.cols*16, 1), 0.0, 0.0, INTER_LINEAR);
                            resize(template_img, template_resized, Size(template_img.cols*16, 1), 0.0, 0.0, INTER_LINEAR);
                            
                            // Perform template matching
                            matchTemplate(roi_resized, template_resized, match_result, TM_SQDIFF_NORMED);
                            minMaxLoc(match_result, &min_val, &max_val, &min_loc, &max_loc);                    

                            disparity = 160.0 - min_loc.x/16.0;
                            depth_mm = focal_length_pixel * baseline_mm / disparity;

                            double xmm, ymm, zmm=depth_mm;
                            xmm = zmm * (double)(circ_centre.x - 320) / 854.189844;
                            ymm = zmm * (double)(240 - circ_centre.y) / 856.022388;
                            double distmm = sqrt((pow(xmm,2) + pow(ymm,2) + pow(zmm,2)));
                            

                            // Display stuff **********************************************************************************************
                            circle(rectified_right, Point(circ_centre.x-disparity, circ_centre.y), 5, Scalar(255,0,0), FILLED);
                            putText(rect_l_copy, std::to_string(distmm), Point(boundingBox.x,boundingBox.y), FONT_HERSHEY_PLAIN, 2, Scalar(255,0,0), 2);


                            kiwiPosMsg.point.x = xmm / 1000.0; 
                            kiwiPosMsg.point.y = ymm / 1000.0;  
                            kiwiPosMsg.point.z = zmm / 1000.0;  
                            
                            kiwi_pub.publish(kiwiPosMsg);
                            break; // Dont bother checking the remaining contours, once one fruit is sent
                        }
                    }
                    
                    auto end = std::chrono::high_resolution_clock::now();
                    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
                    total_duration += duration;


                    // Display stuff **********************************************************************************************
                    imshow("Kiwi mask", opening);
                    imshow("Kiwi distances", rect_l_copy);

                    imshow("Left rectified", rectified_left);
                    imshow("Right rectified", rectified_right);

                    key = waitKey(1);
                }

                auto average_duration = total_duration / loops;
                std::cout << std::endl << "Average time: " << average_duration.count() << " us" << std::endl;

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
            PylonTerminate(true);
            std::cout << "Program Ended" << std::endl;

            return EXIT_SUCCESS; // Will be unable to return, corrupted stack and return address. Issue only occurs on Jetson 
        }

    private: 
        // Uses the camera parameters found from matlab calibration to create the rectification maps
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

        // Rounds a float to the nearest integer
        int int_round(float val) {
            return (int)(val + 0.5 - (val<0));
        }

        /* Takes the two rectification maps (returned by initUndistortRectifyMap), one points to the x location, and the other points to the y.
         * Computes a single map (lookup table), which points to the memory offset location of the pixel, instead of the x and y coordinates in image. 
         *
         * Note: was created in order to use the function below (remap_colour) correctly. A breif description of what the function does was given in 
         * (https://trace.tennessee.edu/cgi/viewcontent.cgi?article=2196&context=utk_gradthes) at section 4.2.1, but no code was provided.
         * */
        void precalc_map(const Mat& input_x, const Mat& input_y, std::vector<int>& output_map) {
            // Consistency check, ensures that the mapping 
            CV_Assert(input_x.size() == input_y.size());
            
            // Allocate output memory, if its not the correct size 
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
         * Also optimised to copy each byte of the RGB pixel simultaneously. This is achieved by casting to an int pointer to write 4 bytes at once,
         * and then only shifting along 3 bytes in memory, so that the 4th byte (dummy byte) is overwritten.  
         * Note: original code found at (https://trace.tennessee.edu/cgi/viewcontent.cgi?article=2196&context=utk_gradthes), minor modificaitons
         * have been made to allow it to work on linux. 
         * */
        void remap_color(const Mat& input, Mat& output, std::vector<int>& map) {
            int chunksize = 24; // Equivalent to the number of threads created.
            // Note: the number of pixels in the image must be divisible by this number, else it will create colour shifted bands on the image. 

            // Consistency check. Ensure the 
            CV_Assert(input.isContinuous());
            CV_Assert((input.cols * input.rows) == map.size());

            // Allocate output image memmory, if its not the same size
            if (input.size() != output.size()) {
                output.release();
                output.create(input.size(), input.type());
            }

            // Calculate the memory address ranges that each thread will use 
            int chunk_width = map.size() * 3 / chunksize;
            int map_width = map.size() / chunksize;
            int max_addr_offset = map_width -1;

            /* Begins the threads which apply the mapping transformation to the image.
             * Each thread handles the memory lookup and copy of a given section of the image.
             * The copy opperation makes use of the fact that an uchar pointer is 1 byte and an int pointer is 4 bytes. By 
             * casting between the two types, it is able to copy 4 bytes in one operation, and then shift along 3 bytes to 
             * replace the dummy one in the next operation (since the RGB pixel info is stored in 3 bytes, 1 byte for each 
             * colour). This allows the copy operation to happen much quicker for colour images. See link above for more.  */
            tbb::parallel_for(0, chunksize, [&](int chunk) {
                int m;
                int *mapptr = &(map[0]) + map_width * chunk; // Points to a array of integers. Each integer represents the memory offset of the current pixel
                uchar *data = output.data + chunk_width * chunk; // Points to a matrix of bytes that make up the image. Each pixel is made of 3 sequential byes (RGB)
                int *indata;
                int *maxaddess = mapptr + max_addr_offset;   
                for (; mapptr < maxaddess; ++mapptr) {
                    // Copies a 4-byte section of data (3 bytes for R,G,B, and 1 dummy byte)
                    m = mapptr[0];
                    if (m < 0) { // If the pixel was out of range, set to 0
                        *((int*)data) = 0; 
                    }
                    else { // Otherwise set the pixel to the correct value
                        // indata = (int*)&(input.data[m]);
                        // *((int*)data) = *indata;
                        *((int*)data) = *((int*)&(input.data[m]));
                    }
                    // Increments pointer by 3 bytes, to cause the dummy byte to be overwritten on the next loop
                    data += 3;
                }

                // Copies the last 3 bytes individually, to avoid changing memory beyond what it should
                m = *mapptr;
                if (m < 0) { // If the pixel was out of range, set to 0
                    data[0] = 0;
                    data[1] = 0;
                    data[2] = 0;
                }
                else { // Otherwise set the pixel to the correct value
                    indata = (int*)&(input.data[m]);
                    data[0] = ((uchar*)indata)[0];
                    data[1] = ((uchar*)indata)[1];
                    data[2] = ((uchar*)indata)[2];
                }
            });

            return;
        }

        
};




volatile bool keepRunning = true;

// Signal handler function
void signalHandler(int signum) {
    // std::cerr << "Interrupt signal (" << signum << ") received." << std::endl;

    // Set the flag to terminate the loop
    keepRunning = false;
}

int main(int argc, char **argv) {
    // Register signal handler for SIGINT
    std::signal(SIGINT, signalHandler);

    Baslers camera_pair(&keepRunning);
    camera_pair.main_program(); // Its unable to return from here due to errors on the jetson, cant do anything afterwards
    
    // Never gets to here, when compiling and running on the jetson
    std::cout << "Never reaches this print, due to return errors" << std::endl;
    return EXIT_SUCCESS; 
}