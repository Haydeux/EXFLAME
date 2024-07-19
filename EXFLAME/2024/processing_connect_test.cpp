/* Title:       Baslers Class
 * Author:      Hayden Moxsom
 * Modified:    12/02/2024 (February 12th)
 * Description: A semi-class based implementation of the ros_sender code. This program connects to two basler cameras, performs rectification,
 *              processes the image to extract a kiwifruit, performs stereo matching on this feature, and finally sends the position of the fruit
 *              in mm relative to the camera through a ROS message.  
 * */

#define DISPLAY_IMAGES 1    // Set 1 to see the images, set 0 for no images
                            // Runs about 110 Hz with images, about 265 Hz with no images

// 265 Hz with no image display, about 110 Hz when displaying

// Opencv library(s) (requires install. CUDA support is currently unused, so its optional)
#include <opencv2/opencv.hpp>
// #include <opencv2/cudawarping.hpp>
// #include <opencv2/cudaimgproc.hpp>
// #include <opencv2/cudafilters.hpp>

// Pylon cameras library (requires install alongside Pylon Camera Software Suite)
#include <pylon/PylonIncludes.h>

// Standard libraries (installed by default)
#include <iostream>
#include <csignal>
#include <cstdlib>
#include <chrono>
#include <sstream>
#include <thread>

// Threading library (install using: 'sudo apt install libtbb-dev'. See oneAPI threading building blocks (oneTBB))
#include <tbb/parallel_for.h>

// Ros libraries (requires install. Currently using ros noetic)
#include <ros/ros.h> 
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Polygon.h>

// Namespaces for opencv (cv) and pylon cameras (Pylon)
using namespace cv;
using namespace Pylon;


class Baslers {
    private:
        volatile bool* m_alive = nullptr; // Safetly stops the program through a ctrl+C press
        volatile bool recieved_rectangles = false;
        std::vector<std::vector<geometry_msgs::Point32>> rectangles_list;
        
    public: 
        // Constructor
        Baslers(volatile bool* alive) { 
            m_alive = alive; // A pointer to a bool flag that gets chnaged by the signal handler, for stopping the program saftely
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

            // Initialise the ros node, message publisher, and message container
            int argc = 0; // Placeholder to pass to the ros init() method
            ros::init(argc, nullptr, "image_processor");

            ros::NodeHandle kiwi_n;
            ros::Publisher kiwi_pub = kiwi_n.advertise<geometry_msgs::PointStamped>("FLAME/KiwiPos", 1);
            geometry_msgs::PointStamped kiwiPosMsg;

            ros::NodeHandle image_n;
            ros::Publisher image_pub = image_n.advertise<sensor_msgs::Image>("image_topic", 1, false);

            ros::NodeHandle rect_n;
            ros::Subscriber rect_sub = rect_n.subscribe("polygons_topic", 1, &Baslers::polygonCallback, this);


            // Initialise the camera software
            PylonInitialize();

            // Get all available camera devices
            CTlFactory& tlFactory = CTlFactory::GetInstance();
            DeviceInfoList_t devices;
            tlFactory.EnumerateDevices(devices);

            // Check to ensure there is exactly 2 cameras
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

            // Initialise a timer #########################################################################
            auto total_duration = std::chrono::microseconds(0);

            // Constant parameters for creating the mask of the kiwifruit
            const int hue_lower=15, hue_upper=38, sat_lower=130, sat_upper=255, val_lower=20, val_upper=230;
            const int radiusClose=0, radiusOpen=1; 
            const int minAreaThreshold=350; 

            // Create the HSV yellow range
            const Scalar lowerYellow(hue_lower, sat_lower, val_lower);
            const Scalar upperYellow(hue_upper, sat_upper, val_upper);

            // Create the opening and closing kernels, for filtering the mask
            const Mat kernel_c = getStructuringElement(MORPH_ELLIPSE, Size(2*radiusClose+1, 2*radiusClose+1));
            const Mat kernel_o = getStructuringElement(MORPH_ELLIPSE, Size(2*radiusOpen+1, 2*radiusOpen+1));
            
            // Initialise a container for the raw data of the captured image
            CGrabResultPtr grabResultRight, grabResultLeft;

            // Initialise Matrices for storing processed images (most operations can't be done in place, so require new variables)
            Mat leftImage(480, 640, CV_8UC1), rightImage(480, 640, CV_8UC1);
            Mat color_left_image(480, 640, CV_8UC3), color_right_image(480, 640, CV_8UC3);
            Mat rectified_left(480, 640, CV_8UC3), rectified_right(480, 640, CV_8UC3);
            
            // Initiliase variables necessary for extracting the kiwifruit from the images
            Mat hsvImage(480, 640, CV_8UC3), yellowMask, closing, opening;
            std::vector<std::vector<Point>> contours;

            // Initialise some matrices for storing the template matching images
            Mat roi_img, template_img, roi_resized, template_resized, match_result;

#if DISPLAY_IMAGES == 1
            // Initialise a variable for holding a copy of the image. For display purposes only ***********************************************
            Mat rect_l_copy(480, 640, CV_8UC3);
#endif

            // Main loop starting
            std::cout << "Program starting" << std::endl;

            try {
                // Open the cameras
                camera_left.Open();
                camera_right.Open();

                // Start grabbing images for the cameras, only grabbing the most recently taken image (ignores the buffer)
                camera_left.StartGrabbing(GrabStrategy_LatestImageOnly);
                camera_right.StartGrabbing(GrabStrategy_LatestImageOnly);

                // Continue running as long as the cameras are still grabbing, ros is still running, and a stop command has not been recieved
                unsigned long loops = 0; // For timing ###########################################
                int key = 0;
                while (camera_left.IsGrabbing() && camera_right.IsGrabbing() && key != 27 && *m_alive && ros::ok()) {

                    // For timing ######################################################################
                    loops++;
                    auto start = std::chrono::high_resolution_clock::now();

                    // Grab and retrieve images from camera_left
                    camera_left.RetrieveResult(500, grabResultLeft, TimeoutHandling_ThrowException);
                    if (grabResultLeft->GrabSucceeded()) {
                        leftImage.data = static_cast<uchar*>(grabResultLeft->GetBuffer());
                    } else continue; // If no image was captured, skip this loop
                    
                    
                    // Grab and retrieve images from camera_right
                    camera_right.RetrieveResult(500, grabResultRight, TimeoutHandling_ThrowException);
                    if (grabResultRight->GrabSucceeded()) {
                        rightImage.data = static_cast<uchar*>(grabResultRight->GetBuffer());
                    } else continue; // If no image was captured, skip this loop

                    // Record the time that the images were captured at, to be sent in the ros message
                    kiwiPosMsg.header.stamp = ros::Time::now();

                    // Convert from bayer BG colour scheme to RGB colour
                    cvtColor(leftImage, color_left_image, COLOR_BayerBG2RGB);
                    cvtColor(rightImage, color_right_image, COLOR_BayerBG2RGB);
                    
                    // Rectify the image
                    remap_color(color_left_image, rectified_left, map_left);
                    remap_color(color_right_image, rectified_right, map_right);

#if DISPLAY_IMAGES == 1
                    // Display stuff -- image copy for displaying on
                    rectified_left.copyTo(rect_l_copy);
                    circle(rect_l_copy, Point(320, 240), 5, Scalar(0,0,255), FILLED);
                    line(rect_l_copy, Point(320, 0), Point(320, 479), Scalar(0,0,255), 1); // Vertical line
                    line(rect_l_copy, Point(0, 240), Point(639, 240), Scalar(0,0,255), 1); // Horizontal line
#endif
                    
                    //ros send
                    sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rectified_left).toImageMsg();
                    image_msg->header.stamp = ros::Time::now();
                    image_pub.publish(image_msg);

                    while (ros::ok() && !recieved_rectangles) {
                        ros::spinOnce();
                    }
                    recieved_rectangles = false;
                   
                    // Sort through all the found contours
                    for (const auto& rectang : rectangles_list) {
                        // Create a rectangle around the detected fruit and find its centre
                        const Rect boundingBox(int(rectang[0].x), int(rectang[0].y), int(rectang[1].x-rectang[0].x), int(rectang[1].y-rectang[0].y)); 
                        const Point circ_centre(boundingBox.x+boundingBox.width/2, boundingBox.y+boundingBox.height/2);
                        
                        // Calculates the horizontal region in which to search for a matching kiwifruit 
                        const int x_start = boundingBox.x - 160;
                        const int x_width = boundingBox.width + 150;
                        // Check if the region is within the bounds of the image, skipping it if its not valid
                        if (x_start + x_width >= rectified_left.cols) continue;
                        if (x_start < 0) continue;

                        // Crop the images to the region of interest to be used for template matching. 
                        // Takes a 1 pixel horizontal slice from the centre of the region, as it majorly increases speed at very little cost to accuracy 
                        const Rect roi_match(x_start, circ_centre.y, x_width, 1); // Slice of the region to search in for a match
                        const Rect roi_template(boundingBox.x, circ_centre.y, boundingBox.width, 1); // Slice of detected kiwifruit

                        //std::cout << x_start <<" | "<< circ_centre.y <<" | "<< x_width << std::endl;

                        // Crop the images, the left into just the detected kiwifruit, the right into the region to search for a match
                        roi_img = rectified_right(roi_match);
                        template_img = rectified_left(roi_template);

                        // Stretch the image to be 16 times as wide. Intended to allow for sub-pixel accuracy. Not sure how well it works
                        resize(roi_img, roi_resized, Size(roi_img.cols*16, 1), 0.0, 0.0, INTER_LINEAR);
                        resize(template_img, template_resized, Size(template_img.cols*16, 1), 0.0, 0.0, INTER_LINEAR);
                        
                        // Perform template matching
                        matchTemplate(roi_resized, template_resized, match_result, TM_SQDIFF_NORMED);
                        Point min_loc;
                        minMaxLoc(match_result, nullptr, nullptr, &min_loc, nullptr);                    

                        // Convert the disparity back into the correct scale
                        const double disparity = 160.0 - min_loc.x/16.0;

                        // Calculate the depth in mm -- Change to try Q matrix
                        const double zmm = focal_length_pixel * baseline_mm / disparity;
                        // Change the position from pixel to mm -- Change this to try using the Q matrix (potentially more accurate)
                        const double xmm = zmm * (double)(circ_centre.x - 320) / 854.189844;
                        const double ymm = zmm * (double)(240 - circ_centre.y) / 856.022388;
                        const double distmm = sqrt((pow(xmm,2) + pow(ymm,2) + pow(zmm,2)));
                        
#if DISPLAY_IMAGES == 1     
                        // Display stuff - box around fruit and centre dot ***************************************************************************
                        rectangle(rect_l_copy, boundingBox, Scalar(0, 255, 0), 2); 
                        circle(rect_l_copy, circ_centre, 5, Scalar(255,0,0), FILLED);
                        // Display stuff, detected circle and distance text **********************************************************************************************
                        circle(rectified_right, Point(circ_centre.x-disparity, circ_centre.y), 5, Scalar(255,0,0), FILLED);
                        putText(rect_l_copy, std::to_string(distmm), Point(boundingBox.x,boundingBox.y), FONT_HERSHEY_PLAIN, 2, Scalar(255,0,0), 2);
                        // vertical and horizontal 'ruler' lines at fruit centre  *******************************************************************
                        line(rect_l_copy, Point(circ_centre.x, 0), Point(circ_centre.x, 479), Scalar(255,0,0), 1); // Vertical line
                        line(rect_l_copy, Point(0, circ_centre.y), Point(639, circ_centre.y), Scalar(255,0,0), 1); // Horizontal line
#endif

                        // Convert the measurements into metres for the ros message
                        kiwiPosMsg.point.x = xmm / 1000.0; 
                        kiwiPosMsg.point.y = ymm / 1000.0;  
                        kiwiPosMsg.point.z = zmm / 1000.0;  
                        
                        // Publish the ros message
                        kiwi_pub.publish(kiwiPosMsg);
                    }

                    // Calculate loop time taken ######################################################################################
                    auto end = std::chrono::high_resolution_clock::now();
                    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
                    total_duration += duration;
                    
#if DISPLAY_IMAGES == 1
                    // Display stuff **********************************************************************************************
                    imshow("Kiwi distances", rect_l_copy);
                    imshow("Left rectified", rectified_left);
                    imshow("Right rectified", rectified_right);
                    key = waitKey(1);
#endif
                }
                
                // Print the average time taken by the loop  ###################################################################################
                auto average_duration = total_duration / loops;
                std::cout << std::endl << "Average time: " << average_duration.count() << " us" << std::endl;

            } // Catch any generic pylon camera exceptions
            catch (const GenericException &e) { 
                std::cerr << "An exception occurred: " << e.GetDescription() << std::endl;
                return -1;
            }

#if DISPLAY_IMAGES == 1
            // Close all image windows
            destroyAllWindows();
#endif

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
        int get_rectifier_maps(double a, Mat& map_left_x, Mat& map_left_y, Mat& map_right_x, Mat& map_right_y, Rect& roi_left, Rect& roi_right) {
            // Create camera parameters - these come from matlab stereo calibration
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

            // Create stereo rectification parameters
            Mat R1, R2, P1, P2, Q;
            stereoRectify(
                intrinsics_left, distortion_left,
                intrinsics_right, distortion_right,
                Size(640, 480),  // Size of the images - camera has a 640x480 resolution
                extrinsics_R, extrinsics_T,
                R1, R2, P1, P2, Q,
                CALIB_ZERO_DISPARITY, a, Size(640, 480), &roi_left, &roi_right
            );

            // Compute the rectification maps
            initUndistortRectifyMap(
                intrinsics_left, distortion_left, R1, P1,
                Size(640, 480), CV_32F, map_left_x, map_left_y
            );
            initUndistortRectifyMap(
                intrinsics_right, distortion_right, R2, P2,
                Size(640, 480), CV_32F, map_right_x, map_right_y
            );

            return 0; // Successful 
        }

        // Rounds a float to the nearest integer
        int int_round(float val) {
            return int(val + 0.5 - (val<0));
        }

        /* Takes the two rectification maps (returned by initUndistortRectifyMap), one points to the x location, and the other points to the y.
         * Computes a single map (lookup table), which points to the memory offset location of the pixel, instead of the x and y coordinates in image. 
         *
         * Note: was created in order to use the function below (remap_colour) correctly. A breif description of what the function does was given in 
         * (https://trace.tennessee.edu/cgi/viewcontent.cgi?article=2196&context=utk_gradthes) at section 4.2.1, but no code was provided.
         * */
        int precalc_map(const Mat& input_x, const Mat& input_y, std::vector<int>& output_map) {
            // Consistency check, ensures that the x and y maps are matching sizes 
            CV_Assert(input_x.size() == input_y.size());
            
            // Allocate output memory, if its not the correct size already
            if (output_map.size() != (input_x.cols * input_x.rows)) {
                output_map.resize((input_x.cols * input_x.rows));
            }
            
            // Loop through the rectification map, calculating the memory offset of the pixel from the unrectified image
            int idx=0;
            for (int i=0; i<input_x.rows; i++) { // Rows first due to row major order 
                for (int j=0; j<input_x.cols; j++) { // Columns
                    // At location (i, j) in the rectified image, determine the nearest corresponding pixel (x, y) from the raw image
                    int y_coord = int_round(input_y.at<float>(i,j));
                    int x_coord = int_round(input_x.at<float>(i,j));

                    // Check if the pixel (x, y) from the raw image is valid
                    if ((x_coord < 0) || (x_coord >= input_x.cols) || (y_coord < 0) || (y_coord >= input_x.rows)) {
                        output_map[idx] = -1; // Its invalid, this will become a black border pixel
                    }
                    else { 
                        // Its valid, calculate the memory offset location. ( y * num_cols + x ) Since row major order (see: https://en.wikipedia.org/wiki/Row-_and_column-major_order)
                        output_map[idx] = (y_coord * input_x.cols + x_coord) * 3; // Times 3 as there is 3 bytes per pixel (1 byte for each colour (R, G, B))
                    }
                    
                    idx++;
                }
            }

            return 0; // Successful
        }

        /* Optimised remapping function which utilises a lookup table pointing to the pixel memory offset, which allows quicker computation.
         * Also optimised to copy each byte of the RGB pixel simultaneously. This is achieved by casting to an int pointer to write 4 bytes at once,
         * and then only shifting along 3 bytes in memory, so that the 4th byte (dummy byte) is overwritten.  
         * Note: original code found at (https://trace.tennessee.edu/cgi/viewcontent.cgi?article=2196&context=utk_gradthes), minor modificaitons
         * have been made to allow it to work on linux. 
         * */
        int remap_color(const Mat& input, Mat& output, std::vector<int>& map) {
            int chunksize = 24; // Equivalent to the number of threads created.
            // Note: the number of pixels in the image must be divisible by this number, else it will create colour shifted bands on the image. 

            // Consistency check. Ensure the image is the correct size 
            CV_Assert(input.isContinuous());
            CV_Assert((input.cols * input.rows) == map.size());

            // Allocate output image memmory, if its not the same size already
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
                int m; // Value by which the memory is offset from the first byte 
                int *mapptr = &(map[0]) + map_width * chunk; // Points to a array of integers. Each integer represents the memory offset of the current pixel
                uchar *data = output.data + chunk_width * chunk; // Points to a matrix of bytes that make up the image. Each pixel is made of 3 sequential byes (RGB)
                int *maxaddess = mapptr + max_addr_offset; // Maximum address to be changed by this thread 
                for (; mapptr < maxaddess; ++mapptr) {
                    // Copies a 4-byte section of data (3 bytes for R,G,B, and 1 dummy byte)
                    m = mapptr[0];
                    if (m < 0) { // If the pixel was out of range, set to 0 (for black)
                        *((int*)data) = 0; 
                    }
                    else { // Otherwise set the pixel to the correct value
                        *((int*)data) = *((int*)&(input.data[m])); // Type casting to write multiple bytes in one operation
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
                else { // If the pixel was in range, set the pixel to the correct value
                    int *indata = (int*)&(input.data[m]);
                    data[0] = ((uchar*)indata)[0];
                    data[1] = ((uchar*)indata)[1];
                    data[2] = ((uchar*)indata)[2];
                }
            }); // End parallel_for

            return 0; // Successful
        }

        void polygonCallback(const geometry_msgs::Polygon::ConstPtr& msg) {
            //std::cout << "rects recieved" << std::endl;
            std::vector<std::vector<geometry_msgs::Point32>> rectangles;
            std::vector<geometry_msgs::Point32> current_rectangle;

            for (size_t i = 0; i < msg->points.size(); ++i) {
                current_rectangle.push_back(msg->points[i]);
                if ((i + 1) % 2 == 0) { // Every 2 points make a rectangle
                    rectangles.push_back(current_rectangle);
                    current_rectangle.clear();
                }
            }
            
            rectangles_list = rectangles;
            recieved_rectangles = true;
            //std::cout << "rects processed" << std::endl;
        }

        
};



// Flag indicating if the program has been issued a stop command
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

    // Instatiate the baslers class, pointing to the shutdown flag
    Baslers camera_pair(&keepRunning);

    // Begin the main camera loop
    camera_pair.main_program(); // Its unable to return from here due to errors on the jetson, cant do anything afterwards
    
    // Never gets to here, when compiling and running on the jetson
    std::cout << "Never reaches this print, due to return errors" << std::endl;
    return EXIT_SUCCESS; 
}