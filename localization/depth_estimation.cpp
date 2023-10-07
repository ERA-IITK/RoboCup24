#include <opencv2/opencv.hpp>

int main() {
    // Fisheye camera parameters (you'll need to calibrate your camera for accurate results)
    cv::Mat K = (cv::Mat_<double>(3, 3) << 3.3653753268250244e+02, 0., 7.8648310706841721e+02, 0.,
       3.3483120592273684e+02, 5.9344169001906948e+02, 0., 0., 1.);

    // Fisheye distortion coefficients (also from calibration)
    cv::Mat dist_coeffs = (cv::Mat_<double>(4, 1) << 9.9151573622450404e-02, -3.6377486279912392e-02,
       3.4318237455812539e-03, -7.8032068549697086e-04);

    // Image point coordinates (replace these with your actual image coordinates)
    int x=610, y=457;
    cv::Point2d image_point(x, y);

    // Undistort the image point using the equidistant fisheye model
    cv::Mat undistorted_point;
    cv::fisheye::undistortPoints(cv::Mat(image_point).reshape(2), undistorted_point, K, dist_coeffs);

    // Calculate the depth using the norm of the undistorted point
    double depth = cv::norm(undistorted_point);

    std::cout << "Depth of point (" << x << ", " << y << "): " << depth << " units" << std::endl;

    return 0;
}
