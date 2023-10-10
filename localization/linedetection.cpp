#include <opencv2/opencv.hpp>
#include <vector>

int main() {
    // Load the image
    cv::Mat image = cv::imread("./images/fisheye_field2.jpg");

    if (image.empty()) {
        std::cerr << "Error: Unable to load image." << std::endl;
        return -1;
    }
    //print cv mat image
    
    // Convert the image to HSV color space
    cv::Mat hsv;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
    // std::cout<<hsv;
    // Define the lower and upper bounds for green color (adjust these values as needed)
    cv::Scalar lowerGreen(50, 70, 70); // Lower bound for green in HSV
    cv::Scalar upperGreen(90, 255, 184); // Upper bound for green in HSV

    // Create a mask to isolate green regions
    cv::Mat greenMask;
    cv::inRange(hsv, lowerGreen, upperGreen, greenMask);
    cv::imshow("Green Mask", greenMask);
    cv::waitKey(0);

    // Find contours in the common mask (common regions between green and white)
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(greenMask, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    // Create an output image to draw the contours
    cv::Mat resultImage = image.clone();

    // Draw all detected contours in blue
    cv::drawContours(resultImage, contours, -1, cv::Scalar(255, 0, 0), 1); // Blue color (BGR)

    // Display the result image
    cv::imshow("All Contours Detection", resultImage);
    cv::waitKey(0);
    cv::imwrite("./images/fisheye_field2Contour.jpg", resultImage);
    cv::destroyAllWindows();

    // Print the number of contours detected
    std::cout << "Number of contours detected: " << contours.size() << std::endl;

    return 0;
}
