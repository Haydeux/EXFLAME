#include <opencv2/opencv.hpp>

int main() {
    cv::Mat img = cv::imread("test.png");

    if (img.empty()) {
        std::cerr << "Error: Could not read the image." << std::endl;
        return -1;
    }

    cv::imshow("Image", img);

    cv::waitKey(0);

    cv::destroyAllWindows();

    //img.release();

    return 0;
}