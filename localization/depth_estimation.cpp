#include <opencv2/opencv.hpp>

int main() {
    // Fisheye camera parameters (you'll need to calibrate your camera for accurate results)
    cv::Mat K = (cv::Mat_<double>(3, 3) << 124.47175225,0.,224.71804041,0.,123.73205159,201.10938475,0.,0.,1.);

    // Fisheye distortion coefficients (also from calibration)
    cv::Mat dist_coeffs = (cv::Mat_<double>(4, 1) << 12.6, 7.8, 4.3, 9.8);

    // Image point coordinates (replace these with your actual image coordinates)
    int x=50, y=50;
    cv::Point2d image_point(x, y);

    // Undistort the image point using the equidistant fisheye model
    cv::Mat undistorted_point;
    cv::fisheye::undistortPoints(cv::Mat(image_point).reshape(2), undistorted_point, K, dist_coeffs);

    // Calculate the depth using the norm of the undistorted point
    double depth = cv::norm(undistorted_point);

    std::cout << "Depth of point (" << x << ", " << y << "): " << depth << " units" << std::endl;

    return 0;
}
