#include <opencv2/opencv.hpp>

int main() {
    // Fisheye camera parameters (you'll need to calibrate your camera for accurate results)
    cv::Mat K = (cv::Mat_<double>(3, 3) << 3.3653753268250244e+02, 0., 7.8648310706841721e+02, 0.,
       3.3483120592273684e+02, 5.9344169001906948e+02, 0., 0., 1.);

    // Fisheye distortion coefficients (also from calibration)
    cv::Mat dist_coeffs = (cv::Mat_<double>(4, 1) << 9.9151573622450404e-02, -3.6377486279912392e-02,
       3.4318237455812539e-03, -7.8032068549697086e-04);

    // Create a matrix to store depth values
    cv::Mat data(610, 456, CV_32F);  // Use CV_32F for 32-bit floating-point data

    // Calculate depth values for each pixel
    for (int i = 0; i < 610; i++) {
        for (int j = 0; j < 456; j++) {
            cv::Point2d image_point(i, j);

            // Undistort the image point using the equidistant fisheye model
            cv::Mat undistorted_point;
            cv::fisheye::undistortPoints(cv::Mat(image_point).reshape(2), undistorted_point, K, dist_coeffs);

            // Calculate the depth using the norm of the undistorted point
            data.at<float>(i, j) = cv::norm(undistorted_point);
        }
    }

    // Scale the depth values to 0-255
    cv::normalize(data, data, 0, 255, cv::NORM_MINMAX);

    // Convert the depth values to 8-bit for colormap
    cv::Mat data_8bit;
    data.convertTo(data_8bit, CV_8U);

    // Apply the colormap
    cv::Mat colormap;
    cv::applyColorMap(data_8bit, colormap, cv::COLORMAP_JET);

    // Display the heatmap
    cv::imshow("Heatmap", colormap);
    cv::waitKey(0);
    cv::destroyAllWindows();

    return 0;
}
