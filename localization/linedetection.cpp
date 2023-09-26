#include <opencv2/opencv.hpp>
#include <vector>

int main() {
    // Load the image
    cv::Mat image = cv::imread("fisheye_field.jpg");

    if (image.empty()) {
        std::cerr << "Error: Unable to load image." << std::endl;
        return -1;
    }

    // Convert the image to grayscale
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    // Threshold the image to create a binary mask of white regions
    cv::Mat binary;
    cv::threshold(gray, binary, 200, 255, cv::THRESH_BINARY);

    // Find contours in the binary image
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

    // Filter and draw the detected white lines (be lenient)
    cv::Mat resultImage = image.clone();
    for (const auto& contour : contours) {
        // Filter white contours (lines) based on area (adjust the area threshold as needed)
        if (cv::contourArea(contour) > 25) { // Adjust the area threshold as needed for line detection
            cv::drawContours(resultImage, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(0, 255, 0), 2); // Draw white contours (lines) in green
        }
    }

    // Display the result
    cv::imshow("White Lines Detection", resultImage);
    cv::waitKey(0);
    cv::destroyAllWindows();

    // Print the number of white lines detected
    std::cout << "Number of white lines detected: " << contours.size() << std::endl;

    return 0;
}
