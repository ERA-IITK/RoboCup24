#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include "localization/raw_points.hpp"
#include "localization/point.hpp"
#include "localization/linepoints.hpp"
#include "localization/realmap_location.hpp"
#include "localization/utilities.hpp"
#include "localization/rndgeneration.hpp"
#include "localization/gradient_descent.hpp"
#include <opencv2/opencv.hpp>
#include "localization/linedetection.hpp"
#include <thread>
#include <memory>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include <cv_bridge/cv_bridge.h>
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using std::placeholders::_1;
using namespace std;
odometry odom;
vector<Eigen::Vector3d> worldCoordinates;
class OdometrySubscriber : public rclcpp::Node
{
  public:
    OdometrySubscriber() : Node("odom_subscriber_node") {
    // Subscribe to the /odom topic
    subscription_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&OdometrySubscriber::odomCallback, this, std::placeholders::_1));
  }

  private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Callback function for /odom topic
    RCLCPP_INFO(get_logger(), "I heard : '%f %f'", msg->pose.pose.position.x, msg->pose.pose.position.y);

    // Extract quaternion orientation from Odometry message
    geometry_msgs::msg::Quaternion orientation_quaternion = msg->pose.pose.orientation;

    // Convert quaternion to Euler angles (roll, pitch, yaw)
    tf2::Quaternion tf_quaternion;
    tf2::fromMsg(orientation_quaternion, tf_quaternion);
    tf2Scalar roll, pitch, yaw;
    tf2::Matrix3x3(tf_quaternion).getRPY(roll, pitch, yaw);

    // Extract rotation around the y-axis (yaw)
    double yaw_degrees = tf2::getYaw(orientation_quaternion) * 180.0 / M_PI;

    // Do something with the yaw angle, e.g., print it
    RCLCPP_INFO(rclcpp::get_logger("odom_listener"), "Yaw angle with respect to y-axis: %f degrees", yaw_degrees);
    odom.x += msg->pose.pose.position.x;
    odom.y += msg->pose.pose.position.y;
    odom.theta += yaw_degrees;
  }

};
class LineSubscriber : public rclcpp::Node
{
  public:
    LineSubscriber() : Node("my_subscriber_node"){
    // Subscribe to the image_raw topic
    image_subscriber_ = create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw",
        10,  // Set the queue size
        std::bind(&LineSubscriber::imageCallback, this, std::placeholders::_1));

    // Subscribe to the camera_info topic
    camera_info_subscriber_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera/camera_info",
        10,  // Set the queue size
        std::bind(&LineSubscriber::cameraInfoCallback, this, std::placeholders::_1));
  }

  private:
    
    Eigen::Matrix3d cameraMatrix;

    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &msg) {
        std::array<double, 9> arrayData = msg->k;
        cameraMatrix(0, 0) = arrayData[0];
        cameraMatrix(0, 1) = arrayData[1];
        cameraMatrix(0, 2) = arrayData[2];
        cameraMatrix(1, 0) = arrayData[3];
        cameraMatrix(1, 1) = arrayData[4];
        cameraMatrix(1, 2) = arrayData[5];
        cameraMatrix(2, 0) = arrayData[6];
        cameraMatrix(2, 1) = arrayData[7];
        cameraMatrix(2, 2) = arrayData[8];
    }
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg) const
    {   
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
        auto contours=linedetection(cv_ptr->image);
        for (const auto& contour : contours) {
            for (const auto& point : contour) {
                // Convert image coordinates to homogeneous coordinates
                Eigen::Vector3d homogeneousCoords(point.x, point.y, 1.0);

                // Use the inverse of the intrinsic matrix to convert to real-world coordinates
                Eigen::Vector3d worldCoords = cameraMatrix.inverse() * homogeneousCoords;

                // Store the result
                worldCoordinates.push_back(worldCoords);
            }
        }
    }
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscriber_;
};
void saveMatrixAsImage(const vector<WPoint> &rwlp)
{
    string filename = "/home/legendarygene/Desktop/RoboCup24/map.png";
    vector<vector<int>> cameraMatrix(220, vector<int>(140, 0));
    vector<WPoint> temp_wlp(rwlp.size());
    vector<WPoint> temp_nlp(rwlp.size());
    for (int ii = 0; ii <= 219; ii += 10)
    {
        for (int jj = 0; jj < 139; jj += 10)
        {
            double a = 1.0 + (double)ii / 10.0, b = 1.0 + (double)jj / 10.0;
            for (int i = 0; i < (int)rwlp.size(); i++)
            {
                realmap_loc(temp_wlp[i].x, temp_wlp[i].y, rwlp[i].x, rwlp[i].y, a, b, 0);
                WPoint temp(temp.x = temp_wlp[i].x, temp.y = temp_wlp[i].y);
                temp.x = max(1.01, temp_wlp[i].x);
                temp.x = min(14.99, temp.x);
                temp.y = max(1.01, temp_wlp[i].y);
                temp.y = min(22.99, temp.y);
                linepoint lp = findNearestPoint(temp);
                temp_nlp[i].x = lp.x;
                temp_nlp[i].y = lp.y;
            }
            double val = costFunction(temp_nlp, temp_wlp);
            // cout << val << "\n";
            cameraMatrix[ii][jj] = 255 - (int)(val * 255);
        }
    }
    cv::Mat image(22, 14, CV_8UC1);
    for (int i = 0; i < 22; ++i)
        for (int j = 0; j < 14; ++j)
            image.at<uchar>(i, j) = static_cast<uchar>(cameraMatrix[i][j]);
    cv::imwrite(filename, image);
    cout << "Saved CostMap in Parent Directory!\n";
}

struct point
{
    double x, y;
};

void localization_thread()
{
    int rndpts = 100;
    // vector<WPoint> rel = {
    //     {5.9, 0}, {5.9, 3.22318}, {2.761, 4.3}, {0.304935, 4.3}, {-1.96793, 4.3}, {-5.75619, 4.3}, {5.9, -0.841024}, {5.9, 2.21006}, {3.71387, 4.3}, {0.927258, 4.3}, {-1.272, 4.3}, {-4.31907, 4.3}, {5.9, -1.71694}, {5.9, 1.29964}, {4.93432, 4.3}, {1.58905, 4.3}, {-0.63238, 4.3}, {-3.24194, 4.3}, {5.9, -2.66866}, {5.9, 0.444649}, {5.9, 3.82533}, {-2.40554, -4.45}, {-0.0190306, 4.3}, {-2.37387, 4.3}, {5.9, -3.75157}, {5.9, -0.392162}, {5.9, 2.73182}, {3.18264, 4.3}, {0.593545, 4.3}, {-1.63246, 4.3}, {5.9, -5.05036}, {5.9, -1.24498}, {5.9, 1.77373}, {4.24328, 4.3}, {1.23071, 4.3}, {-0.967165, 4.3}, {-3.78084, 4.3}, {5.9, -2.1507}, {5.9, 0.894378}, {5.9, 4.48934}, {1.92208, 4.3}, {-0.343211, 4.3}, {4.38626, -6.7}, {5.9, -3.1557}, {5.9, 0.0522246}, {5.9, 3.29133}, {-2.802, -4.45}, {0.266705, 4.3}, {-2.01415, 4.3}, {-5.86373, 4.3}, {5.9, -0.787806}, {5.9, 2.26981}, {3.64792, 4.3}, {0.887501, 4.3}, {-1.3135, 4.3}, {-4.39622, 4.3}, {5.9, -1.66043}, {5.9, 1.3545}, {4.84702, 4.3}, {1.54593, 4.3}, {-0.671315, 4.3}, {-3.30204, 4.3}, {5.9, -2.606}, {5.9, 0.497206}, {5.9, 3.89994}, {2.2755, 4.3}, {-0.0570946, 4.3}, {-2.42378, 4.3}, {5.9, -3.67864}, {5.9, -0.339737}, {5.9, 2.79551}, {-4.55, -6.26258}, {0.554804, 4.3}, {-1.67615, 4.3}, {-5.11435, 4.3}, {5.9, -1.19053}, {5.9, 1.83083}, {4.16881, 4.3}, {1.18963, 4.3}, {-1.00723, 4.3}, {-3.84886, 4.3}, {5.9, -2.09172}, {5.9, 0.947875}, {5.54855, 4.3}, {1.87659, 4.3}, {0.394853, -4.45}, {-2.86975, 4.3}, {5.9, -3.08885}, {5.9, 0.104458}, {5.9, 3.36014}, {2.65469, 4.3}, {0.228517, 4.3}, {-2.06076, 4.3}, {5.9, -4.24681}, {5.9, -0.734713}, {5.9, 2.32996}, {3.58295, 4.3}, {0.84789, 4.3}, {-1.35522, 4.3}, {-4.47478, 4.3}, {5.9, -1.60421}, {5.9, 1.40959}, {4.76145, 4.3}, {1.50309, 4.3}, {-0.71036, 4.3}, {-3.36297, 4.3}, {5.9, -2.54383}, {5.9, 0.549841}, {5.9, 3.97542}, {2.22701, 4.3}, {-0.0951681, 4.3}, {-2.47418, 4.3}, {5.9, -3.60651}, {5.9, -0.287366}, {5.9, 2.85972}, {-3.1733, -4.45}, {0.516154, 4.3}, {-1.72015, 4.3}, {-5.20724, 4.3}, {5.9, -1.13628}, {5.9, 1.88824}, {4.0956, 4.3}, {1.14876, 4.3}, {-1.04747, 4.3}, {-3.91796, 4.3}, {5.9, -2.03312}, {5.9, 1.00152}, {5.9, 4.65653}, {1.83146, 4.3}, {-0.419935, 4.3}, {-2.92509, 4.3}, {5.9, -3.02262}, {5.9, 0.156707}, {5.9, 3.42966}, {-2.69319, -4.45}, {0.190365, 4.3}, {-2.10776, 4.3}, {5.9, -4.16803}, {5.9, -0.681736}, {5.9, 2.39055}, {3.51894, 4.3}, {0.808416, 4.3}, {-1.39718, 4.3}, {-4.5548, 4.3}, {5.9, -1.54826}, {5.9, 1.46491}, {4.67754, 4.3}, {-1.51146, -4.45}, {-0.749517, 4.3}, {-3.42474, 4.3}, {5.9, -2.48214}, {5.9, 0.602562}, {5.9, 4.05181}, {-2.25496, -4.45}, {-0.133256, 4.3}, {-2.52511, 4.3}, {5.9, -3.53516}, {5.9, -0.235041}, {5.9, 2.9245}, {3.00927, 4.3}, {0.477584, 4.3}, {-1.76446, 4.3}, {-5.30213, 4.3}, {5.9, -1.08221}, {5.9, 1.94598}, {4.02361, 4.3}, {1.10808, 4.3}, {-1.08787, 4.3}, {-3.98819, 4.3}, {5.9, -1.97487}, {5.9, 1.05533}, {5.3502, 4.3}, {1.78666, 4.3}, {-0.458394, 4.3}, {-2.98111, 4.3}, {5.9, -2.95698}, {5.9, 0.208981}, {5.9, 3.49989}, {2.55068, 4.3}, {0.152244, 4.3}, {2.23035, -4.45}, {5.9, -4.09022}, {5.9, -0.628868}, {5.9, 2.45156}, {3.45584, 4.3}, {0.769074, 4.3}, {1.4896, -4.45}, {-4.63633, 4.3}, {5.9, -1.49257}, {5.9, 1.52048}, {4.59523, 4.3}, {-1.46765, -4.45}, {-0.788797, 4.3}, {-3.48738, 4.3}, {5.9, -2.4209}, {5.9, 0.655379}, {5.9, 4.12914}, {2.13133, 4.3}, {-0.171366, 4.3}, {-2.57656, 4.3}, {5.9, -3.46456}, {5.9, -0.182751}, {5.9, 2.98984}, {2.95292, 4.3}, {0.439091, 4.3}, {-1.80909, 4.3}, {-5.39912, 4.3}, {5.9, -1.02832}, {5.9, 2.00405}, {3.95281, 4.3}, {1.06758, 4.3}, {-1.12846, 4.3}, {-4.05958, 4.3}, {5.9, -1.91697}, {5.9, 1.10931}, {5.9, 4.82846}, {1.74219, 4.3}, {-0.496924, 4.3}, {-3.03781, 4.3}, {5.9, -2.89193}, {5.9, 0.261288}, {5.9, 3.57086}, {2.49949, 4.3}, {0.114145, 4.3}, {-2.20301, 4.3}, {5.9, -4.01337}, {5.9, -0.5761}, {5.9, 2.51303}, {3.39364, 4.3}, {0.729857, 4.3}, {-1.48184, 4.3}, {5.9, -5.37564}, {5.9, -1.43713}, {5.9, 1.5763}, {4.51447, 4.3}, {-1.42411, -4.45}, {-0.828204, 4.3}, {-3.55094, 4.3}, {5.9, -2.3601}, {5.9, 0.708301}, {5.9, 4.20743}, {2.08413, 4.3}, {-0.209501, 4.3}, {-2.62857, 4.3}, {5.9, -3.39469}, {5.9, -0.130491}, {5.9, 3.05577}, {2.89725, 4.3}, {-0.414642, -4.45}, {-1.85406, 4.3}, {-5.4983, 4.3}, {5.9, -0.974587}, {5.9, 2.06248}, {3.88315, 4.3}, {1.02726, 4.3}, {1.21003, -4.45}, {-4.13217, 4.3}, {5.9, -1.85939}, {5.9, 1.16348}, {5.16039, 4.3}, {1.69804, 4.3}, {-0.535534, 4.3}, {-3.09523, 4.3}, {5.9, -2.82744}, {5.9, 0.313636}, {5.9, 3.6426}, {2.44883, 4.3}, {0.0760651, 4.3}, {-2.25128, 4.3}, {5.9, -3.93744}, {5.9, -0.523423}, {5.9, 2.57496}, {3.3323, 4.3}, {0.690758, 4.3}, {-1.52455, 4.3}, {-4.80417, 4.3}, {5.9, -1.38192}, {5.9, 1.63239}, {4.43519, 4.3}, {1.33426, 4.3}, {-0.867745, 4.3}, {-3.61543, 4.3}, {5.9, -2.29973}, {5.9, 0.761334}, {5.9, 4.28671}, {2.03732, 4.3}, {-0.247671, 4.3}, {-2.68114, 4.3}, {5.9, -3.32553}, {5.9, -0.0782509}, {5.9, 3.12231}, {2.84223, 4.3}, {-0.374943, -4.45}, {-1.89937, 4.3}, {-5.59974, 4.3}, {5.9, -0.921017}, {5.9, 2.12127}, {3.8146, 4.3}, {0.98711, 4.3}, {1.25243, -4.45}, {-4.20601, 4.3}, {5.9, -1.80214}, {5.9, 1.21783}, {5.06849, 4.3}, {1.65419, 4.3}, {-0.57423, 4.3}, {-3.15338, 4.3}, {5.9, -2.76349}, {5.9, 0.366033}, {5.9, 3.71513}, {2.39868, 4.3}, {0.0379972, 4.3}, {-2.3, 4.3}, {5.9, -3.8624}, {5.9, -0.470828}, {5.9, 2.63738}, {3.27179, 4.3}, {0.651769, 4.3}, {-1.56753, 4.3}, {-4.8906, 4.3}, {5.9, -1.32695}, {5.9, 1.68875}, {4.35734, 4.3}, {1.29265, 4.3}, {-0.907428, 4.3}, {-3.68088, 4.3}, {5.9, -2.23978}, {5.9, 0.814489}, {5.80945, 4.3}, {1.99091, 4.3}, {0.295852, -4.45}, {4.2604, -6.7}, {5.9, -3.25706}, {5.9, -0.0260229}, {5.9, 3.18948}, {2.78786, 4.3}, {0.324001, 4.3}, {-1.94504, 4.3}, {-5.70355, 4.3}, {5.9, -0.867593}, {5.9, 2.18043}, {3.74711, 4.3}, {0.947124, 4.3}, {-1.2514, 4.3}, {-4.28114, 4.3}, {5.9, -1.7452}, {5.9, 1.27238}, {5.9, 5.09593}, {1.61065, 4.3}, {-0.613016, 4.3}, {-3.2123, 4.3}, {5.9, -2.70007}, {5.9, 0.418487}, {5.9, 3.78847}, {-2.43096, -4.45}, {-6.48499e-05, 4.3}, {3.66036, -6.7}, {5.9, -3.78822}, {5.9, -0.418308}, {5.9, 2.70029}, {3.2121, 4.3}, {-0.634264, -4.45}, {-1.61079, 4.3}, {-4.97878, 4.3}, {5.9, -1.27219}, {5.9, 1.74539}, {4.28088, 4.3}, {1.25126, 4.3}, {-0.94726, 4.3}, {-3.74734, 4.3}, {5.9, -2.18023}, {5.9, 0.867775}, {5.9, 4.44839}, {1.94488, 4.3}, {-0.324131, 4.3}, {-2.78804, 4.3}, {5.9, -3.18925}, {5.9, 0.0262008}, {5.9, 3.25729}, {2.73411, 4.3}, {0.285749, 4.3}, {-1.99107, 4.3}, {-5.80982, 4.3}, {5.9, -0.814308}, {5.9, 2.23998}, {3.68066, 4.3}, {0.907293, 4.3}, {-1.29279, 4.3}, {-4.35761, 4.3}, {5.9, -1.68855}, {5.9, 1.32713}, {4.8903, 4.3}, {-1.62206, -4.45}, {0.674642, -4.45}, {-3.272, 4.3}, {5.9, -2.63716}, {5.9, 0.471007}, {5.9, 3.86265}, {2.29983, 4.3}, {-0.0381269, 4.3}, {3.73774, -6.7}, {5.9, -3.71488}, {5.9, -0.365853}, {5.9, 2.76371}, {3.15318, 4.3}, {-0.594124, -4.45}, {-1.65434, 4.3}, {-5.0688, 4.3}, {5.9, -1.21764}, {5.9, 1.80234}, {4.20576, 4.3}, {1.21008, 4.3}, {-0.987247, 4.3}, {-3.81483, 4.3}, {5.9, -2.12106}, {5.9, 0.921199}, {5.59939, 4.3}, {1.89921, 4.3}, {-0.362435, 4.3}, {-2.84242, 4.3}, {5.9, -3.12208}, {5.9, 0.0784283}, {5.9, 3.32576}, {2.68095, 4.3}, {0.247541, 4.3}, {-2.03748, 4.3}, {5.9, -4.28644}, {5.9, -0.761154}, {5.9, 2.29994}, {3.61521, 4.3}, {0.867611, 4.3}, {-1.3344, 4.3}, {-4.43545, 4.3}, {5.9, -1.63219}, {5.9, 1.38211}, {4.80388, 4.3}, {1.52441, 4.3}, {-0.690891, 4.3}, {-3.3325, 4.3}, {5.9, -2.57475}, {5.9, 0.523602}, {5.9, 3.9377}, {2.25111, 4.3}, {0.0788536, -4.45}, {-2.449, 4.3}, {5.9, -3.64236}, {5.9, -0.313457}, {5.9, 2.82766}, {-3.203, -4.45}, {0.535403, 4.3}, {-1.69818, 4.3}, {5.9, -4.91599}, {5.9, -1.16329}, {5.9, 1.85959}, {4.13192, 4.3}, {-1.20989, -4.45}, {-1.0274, 4.3}, {-3.88339, 4.3}, {5.9, -2.06228}, {5.9, 0.974771}, {5.9, 4.61444}, {1.8539, 4.3}, {-0.400797, 4.3}, {-2.89744, 4.3}, {5.9, -3.05555}, {5.9, 0.130669}, {5.9, 3.39493}, {2.62839, 4.3}, {0.209373, 4.3}, {-2.08429, 4.3}, {5.9, -4.20716}, {5.9, -0.708121}, {5.9, 2.3603}, {3.55072, 4.3}, {0.828069, 4.3}, {1.42425, -4.45}, {5.9, -5.61937}, {5.9, -1.57611}, {5.9, 1.43732}, {4.71915, 4.3}, {1.48169, 4.3}, {1.13743, -6.7}, {-3.39385, 4.3}, {5.9, -2.51282}, {5.9, 0.57628}, {5.9, 4.01363}, {2.20284, 4.3}, {-0.114275, 4.3}, {-2.49967, 4.3}, {5.9, -3.57062}, {5.9, -0.261109}, {5.9, 2.89215}, {3.03762, 4.3}, {0.496794, 4.3}, {-1.74234, 4.3}, {5.9, -4.82816}, {5.9, -1.10913}, {5.9, 1.91716}, {4.05933, 4.3}, {1.12832, 4.3}, {-1.06772, 4.3}, {-3.95305, 4.3}, {5.9, -2.00385}, {5.9, 1.0285}, {5.39879, 4.3}, {1.80894, 4.3}, {-0.439221, 4.3}, {-2.95311, 4.3}, {5.9, -2.98962}, {5.9, 0.18293}, {5.9, 3.4648}, {2.57639, 4.3}, {0.171236, 4.3}, {-2.13149, 4.3}, {5.9, -4.12887}, {5.9, -0.655199}, {5.9, 2.4211}, {3.48717, 4.3}, {0.788663, 4.3}, {-1.41833, 4.3}, {-4.59551, 4.3}, {5.9, -1.52029}, {5.9, 1.49276}, {4.63605, 4.3}, {1.43924, 4.3}, {-0.769209, 4.3}, {-3.45605, 4.3}, {5.9, -2.45135}, {5.9, 0.629048}, {5.9, 4.09049}, {-2.23018, -4.45}, {-0.152373, 4.3}, {-2.55085, 4.3}, {5.9, -3.49965}, {5.9, -0.208803}, {5.9, 2.95721}, {-4.55, -6.56342}, {0.458262, 4.3}, {-1.78681, 4.3}, {-5.35053, 4.3}, {5.9, -1.05515}, {5.9, 1.97507}, {3.98795, 4.3}, {1.08774, 4.3}, {1.14687, -4.45}, {-4.02386, 4.3}, {5.9, -1.94578}, {5.9, 1.08239}, {5.30181, 4.3}, {1.76431, 4.3}, {-0.477715, 4.3}, {-3.00947, 4.3}, {5.9, -2.92428}, {5.9, 0.235219}, {5.9, 3.5354}, {2.52493, 4.3}, {0.133126, 4.3}, {-2.17912, 4.3}, {5.9, -4.05155}, {5.9, -0.602383}, {5.9, 2.48234}, {3.42452, 4.3}, {0.749384, 4.3}, {-1.46065, 4.3}, {5.9, -5.42346}, {5.9, -1.46472}, {5.9, 1.54845}, {4.55452, 4.3}, {1.39704, 4.3}, {-0.808551, 4.3}, {-3.51915, 4.3}, {5.9, -2.39034}, {5.9, 0.681916}, {5.9, 4.16829}, {2.1076, 4.3}, {0.263592, -5.95}, {4.05519, -6.7}, {5.9, -3.42942}, {5.9, -0.156529}, {5.9, 3.02284}, {-3.02694, -4.45}, {0.419804, 4.3}, {-1.83161, 4.3}, {5.9, -4.65624}, {5.9, -1.00134}, {5.9, 2.03332}, {3.91772, 4.3}, {1.04733, 4.3}, {-1.1489, 4.3}, {-4.09585, 4.3}, {5.9, -1.88804}, {5.9, 1.13647}, {5.20692, 4.3}, {1.72, 4.3}, {-0.516285, 4.3}, {-3.06653, 4.3}, {5.9, -2.85951}, {5.9, 0.287545}, {5.9, 3.60676}, {-2.56031, -4.45}, {-0.131508, -5.95}, {2.30486, -4.45}, {5.9, -3.97516}, {5.9, -0.549661}, {5.9, 2.54404}, {3.36276, 4.3}, {0.710226, 4.3}, {-1.50323, 4.3}, {-4.76174, 4.3}, {5.9, -1.4094}, {5.9, 1.6044}, {4.47451, 4.3}, {-1.40235, -4.45}, {-0.848024, 4.3}, {-3.58317, 4.3}, {5.9, -2.32976}, {5.9, 0.734893}, {5.9, 4.24708}, {2.0606, 4.3}, {-0.228646, 4.3}, {-2.65487, 4.3}, {5.9, -3.35991}, {5.9, -0.10428}, {5.9, 3.08908}, {2.86957, 4.3}, {0.381413, 4.3}, {-1.87675, 4.3}, {5.9, -4.57208}, {5.9, -0.947692}, {5.9, 2.09193}, {3.84862, 4.3}, {1.0071, 4.3}, {1.23128, -4.45}, {-4.16906, 4.3}, {5.9, -1.83063}, {5.9, 1.19072}, {5.11404, 4.3}, {1.676, 4.3}, {-0.554937, 4.3}, {4.86811, -6.7}, {5.9, -2.79529}, {5.9, 0.339916}, {5.9, 3.67889}, {-2.50815, -4.45}, {0.0569658, 4.3}, {-2.27567, 4.3}, {5.9, -3.89968}, {5.9, -0.497026}, {5.9, 2.60622}, {3.30184, 4.3}, {0.671183, 4.3}, {-1.54608, 4.3}, {-4.84732, 4.3}, {5.9, -1.35431}, {5.9, 1.66063}, {-4.5493, -4.45}, {1.31336, 4.3}, {-0.887636, 4.3}, {-3.64814, 4.3}, {5.9, -2.2696}, {5.9, 0.787987}, {5.86336, 4.3}, {2.01399, 4.3}, {-0.266835, 4.3}, {-2.70773, 4.3}, {5.9, -3.29109}, {5.9, -0.0520473}, {5.9, 3.15593}, {2.81488, 4.3}, {0.343081, 4.3}, {-1.92223, 4.3}, {-5.65152, 4.3}, {5.9, -0.894196}, {5.9, 2.1509}, {3.78061, 4.3}, {0.967029, 4.3}, {-1.23085, 4.3}, {-4.24354, 4.3}, {5.9, -1.77354}, {5.9, 1.24517}, {5.9, 5.05067}, {1.63231, 4.3}, {-0.593678, 4.3}, {-3.18284, 4.3}, {5.9, -2.73161}, {5.9, 0.392342}, {5.9, 3.75182}, {-2.45651, -4.45}, {0.0189018, 4.3}, {-2.32462, 4.3}, {5.9, -3.82508}, {5.9, -0.44447}, {5.9, 2.66888}, {3.24174, 4.3}, {0.632248, 4.3}, {-1.5892, 4.3}, {-4.93462, 4.3}, {5.9, -1.29945}, {5.9, 1.71713}, {4.31881, 4.3}, {1.27185, 4.3}, {-0.927392, 4.3}, {-3.7141, 4.3}, {5.9, -2.20985}, {5.9, 0.841207}, {5.9, 4.40771}, {1.96777, 4.3}, {-0.305064, 4.3}, {-2.76118, 4.3}, {5.9, -3.22295}, {5.9, 0.00017786}, {5.9, 3.22342}, {2.76082, 4.3}, {-0.315435, -4.45}, {-1.96808, 4.3}, {-5.75655, 4.3}, {5.9, -0.840843}, {5.9, 2.21026}, {3.71365, 4.3}, {0.927122, 4.3}};

    // // cout<<"Enter number of points: ";
    // // cin >> rndpts;
    // // vector<raw_pt> raw_wlp = rndgen2(50);
    // // saveMatrixAsImage(rel);

    vector<Point> pts = rndgen(rndpts);

    // Point act;
    // act.x = 9.1, act.y = 7.7, act.theta = 0;
    // pts.push_back(act);

    for (int i = 0; i < rndpts; i++)
    {
        // pts[i].rwlp.resize(raw_wlp.size());
        // pts[i].wlp.resize(raw_wlp.size());
        // pts[i].nlp.resize(raw_wlp.size());

        pts[i].rwlp.resize(worldCoordinates.size());
        pts[i].wlp.resize(worldCoordinates.size());
        pts[i].nlp.resize(worldCoordinates.size());

        for (int j = 0; j < worldCoordinates.size(); j++)
        {
            // get_projection(raw_wlp[j].depth, raw_wlp[j].x_angle, raw_wlp[j].y_angle, pts[i].rwlp[j].x, pts[i].rwlp[j].y);
            pts[i].rwlp[j].x = worldCoordinates[j].x();
            pts[i].rwlp[j].y = worldCoordinates[j].y();
        }
    }

    for (int i = 0; i < rndpts; i++)
    {
        gradient_descent(pts[i]);
    }

    sort(pts.begin(), pts.end(), [](const Point &lhs, const Point &rhs)
         { return lhs.cost < rhs.cost; });

    // pts.resize(100);
    for (int i = 0; i < rndpts; i++)
    {
        cout << pts[i].x << " " << pts[i].y << " " << pts[i].theta << "\n";
    }
    int frames = 2;
    while(frames--){
        update_odom(pts, odom);
        odom.x = 0, odom.y = 0, odom.theta = 0;
        inc_age(pts);
        addscore(pts);
        del_nodes(pts);

        for (int i = 0; i < rndpts; i++)
        {
            // pts[i].rwlp.resize(raw_wlp.size());
            // pts[i].wlp.resize(raw_wlp.size());
            // pts[i].nlp.resize(raw_wlp.size());

            pts[i].rwlp.resize(worldCoordinates.size());
            pts[i].wlp.resize(worldCoordinates.size());
            pts[i].nlp.resize(worldCoordinates.size());

            for (int j = 0; j < worldCoordinates.size(); j++)
            {
                // get_projection(raw_wlp[j].depth, raw_wlp[j].x_angle, raw_wlp[j].y_angle, pts[i].rwlp[j].x, pts[i].rwlp[j].y);
                pts[i].rwlp[j].x = worldCoordinates[j].x();
                pts[i].rwlp[j].y = worldCoordinates[j].y();
            }
        }

        for (int i = 0; i < rndpts; i++)
        {
            gradient_descent(pts[i]);
        }

        sort(pts.begin(), pts.end(), [](const Point &lhs, const Point &rhs)
            { return lhs.cost < rhs.cost; });

        // pts.resize(100);
        for (int i = 0; i < rndpts; i++)
        {
            cout << pts[i].x << " " << pts[i].y << " " << pts[i].theta << "\n";
        }
    }
}

int odometry_thread(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometrySubscriber>());
  rclcpp::shutdown();
  return 0;
}
int linedetection_thread(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LineSubscriber>());
  rclcpp::shutdown();
  return 0;
}

int main(int argc, char * argv[])
{
    odom.x = 0;
    odom.y = 0;
    odom.theta = 0;
    thread t1(odometry_thread, argc, argv);
    thread t2(localization_thread);
    thread t3(linedetection_thread, argc, argv);
    t1.join();
    t2.join();
    t3.join();
    return 0;
}


