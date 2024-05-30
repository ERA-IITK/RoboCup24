#include <iostream>
#include <fstream>
#include <vector>
#include <deque>
#include <cmath>
#include <algorithm>
#include "localization/raw_points.hpp"
#include "localization/point.hpp"
#include "localization/linepoints_ambition.hpp"//ambition
#include "localization/realmap_location.hpp"
#include "localization/utilities.hpp"
#include "localization/rndgeneration_ambition.hpp"//ambition
#include "localization/gradient_descent_ambition.hpp"//ambition
#include <opencv2/opencv.hpp>
#include "localization/linedetection.hpp"
//#include "localization/LevenbergMarquardt.hpp"
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
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <chrono>

using std::placeholders::_1;
using namespace std;
odometry odom;
vector<Eigen::Vector3d> worldCoordinates;
deque<vector<Eigen::Vector3d>> worldCoordinatesDeque;
ofstream myfile;
int counter = 0; // For camera points testing

void displayPoints(cv::Mat& image, const std::vector<cv::Point2f>& points, const std::vector<double>& thetas_degrees)
{
    for (size_t i = 0; i < points.size(); ++i)
    {
        const auto& point = points[i];
        const double theta_degrees = thetas_degrees[i];
        const double theta_radians = -1*(theta_degrees*M_PI)/180.0;

        // Draw circle and arrow
        cv::Scalar color = (i == 0) ? cv::Scalar(255, 0, 0) : cv::Scalar(0, 255, 255);
        cv::circle(image, point, 5, color, -1);
        int arrowLength = 30;
        cv::Point2f arrowEnd(static_cast<float>(point.x + arrowLength * std::cos(theta_radians)), static_cast<float>(point.y + arrowLength * std::sin(theta_radians)));
        cv::arrowedLine(image, point, arrowEnd, color, 2);
    }
    cv::imshow("Image with Points", image);
    cv::waitKey(1000);
}

class OdometrySubscriber : public rclcpp::Node
{
public:
    OdometrySubscriber() : Node("odom_subscriber_node")
    {
        subscription_ = create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&OdometrySubscriber::odomCallback, this, std::placeholders::_1));
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // RCLCPP_INFO(get_logger(), "I heard : '%f %f'", msg->pose.pose.position.x, msg->pose.pose.position.y);

        // Extract quaternion orientation from Odometry message
        geometry_msgs::msg::Quaternion orientation_quaternion = msg->pose.pose.orientation;
        tf2::Quaternion tf_quaternion;
        tf2::fromMsg(orientation_quaternion, tf_quaternion);
        tf2Scalar roll, pitch, yaw;
        tf2::Matrix3x3(tf_quaternion).getRPY(roll, pitch, yaw);

        // Extract rotation around the y-axis (yaw)
        double yaw_degrees = tf2::getYaw(orientation_quaternion) * 180.0 / M_PI;
        // RCLCPP_INFO(rclcpp::get_logger("odom_listener"), "Yaw angle with respect to y-axis: %f degrees", yaw_degrees);
        odom.x = msg->pose.pose.position.x;
        odom.y = msg->pose.pose.position.y;
        odom.theta = yaw_degrees;
    }
};

class LineSubscriber : public rclcpp::Node
{
public:
    LineSubscriber(const std::string &camera_name, const std::string &camera_frame) 
        : Node(camera_name + "_subscriber_node"),
          tf_buffer_(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)),
          tf_listener_(tf_buffer_),
          camera_frame_(camera_frame)
    {
        image_subscriber_ = create_subscription<sensor_msgs::msg::Image>("/" + camera_name + "/image_raw", 10, std::bind(&LineSubscriber::imageCallback, this, std::placeholders::_1));
        depth_subscriber_ = create_subscription<sensor_msgs::msg::Image>("/" + camera_name + "/depth/image_raw", 10, std::bind(&LineSubscriber::depthCallback, this, std::placeholders::_1));
        camera_info_subscriber_ = create_subscription<sensor_msgs::msg::CameraInfo>("/" + camera_name + "/camera_info", 10, std::bind(&LineSubscriber::cameraInfoCallback, this, std::placeholders::_1));
    }

private:
    Eigen::Matrix3d cameraMatrix;
    std::vector<std::vector<cv::Point>> contours;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::string camera_frame_;

    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &msg)
    {
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

    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
        contours = linedetection(cv_ptr->image);
    }

    void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        const float *depth_data = reinterpret_cast<const float *>(msg->data.data());
        int width = msg->width;
        int height = msg->height;

        for (const auto &contour : contours)
        {
            for (const auto &point : contour)
            {
                float depth_value = depth_data[point.y * width + point.x];
                Eigen::Vector3d homogeneousCoords(point.x, point.y, 1.0);
                Eigen::Vector3d worldCoords = cameraMatrix.inverse() * homogeneousCoords;

                worldCoords.x() /= worldCoords.z();
                worldCoords.y() /= worldCoords.z();
                worldCoords.z() /= worldCoords.z();
                worldCoords.x() *= depth_value;
                worldCoords.y() *= depth_value;
                worldCoords.z() *= depth_value;

                // Transform point from camera frame to robot frame
                geometry_msgs::msg::PointStamped point_camera_frame;
                point_camera_frame.header.frame_id = camera_frame_;
                point_camera_frame.point.x = worldCoords.x();
                point_camera_frame.point.y = worldCoords.y();
                point_camera_frame.point.z = worldCoords.z();

                try
                {
                    geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_.lookupTransform("base_link", camera_frame_, tf2::TimePointZero);
                    geometry_msgs::msg::PointStamped point_robot_frame;
                    tf2::doTransform(point_camera_frame, point_robot_frame, transform_stamped);

                    // Store the result
                    double thee = (odom.theta * M_PI) / 180.0;
                    myfile << "(" << point.x << ", " << point.y << ") is (" << odom.x + point_robot_frame.point.x << "," << odom.y + point_robot_frame.point.y << ")\n";
                }
                catch (tf2::TransformException &ex)
                {
                    RCLCPP_WARN(this->get_logger(), "Could not transform point: %s", ex.what());
                }

                worldCoordinates.push_back(worldCoords);
            }
        }
        if (worldCoordinatesDeque.size() < 10)
        {
            worldCoordinatesDeque.push_back(worldCoordinates);
        }
        else
        {
            worldCoordinatesDeque.pop_front();
            worldCoordinatesDeque.push_back(worldCoordinates);
        }
        worldCoordinates.clear();
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscriber_;
};



struct point
{
    double x, y;
};

void localization_thread()
{
    int rndpts = 50;
    vector<Point> pts = rndgen(rndpts);

    // Point one, two, three, four, five;
    // one.x = 0.01, one.y = 0.02, one.theta = 1;
    // two.x = 0.02, one.y = -0.03, one.theta = 3;
    // three.x = 0, one.y = 0.01, one.theta = -2;
    // four.x = -0.01, one.y = 0, one.theta = -1;
    // five.x = 0.02, one.y = 0, one.theta = 0;
    // vector<Point> pts = {one, two, three, four, five};

    odometry temp;
    temp.x = 0, temp.y = 0, temp.theta = 0;
    int k = 0;
    std::string imagePath = "/home/vihaan/Documents/RoboCup24/localization_ws/src/localization/images/robocup_field_ambition.png";
    cv::namedWindow("Image with Points", cv::WINDOW_NORMAL);

    while (true)
    {
        k++;
        vector<Eigen::Vector3d> tempWorldCoordinates = worldCoordinatesDeque.front();
        // cout<<odom.x<<" "<<odom.y<<" "<<odom.theta<<"\n";
        temp.x = odom.x - temp.x;
        temp.y = odom.y - temp.y;
        temp.theta = odom.theta - temp.theta;
        update_odom(pts, temp);
        temp.x = odom.x;
        temp.y = odom.y;
        temp.theta = odom.theta;
        // inc_age(pts);
        // addscore(pts);
        // del_nodes(pts);

        for (int i = 0; i < rndpts; i++)
        {
            pts[i].rwlp.resize(tempWorldCoordinates.size());
            pts[i].wlp.resize(tempWorldCoordinates.size());
            pts[i].nlp.resize(tempWorldCoordinates.size());

            for (int j = 0; j < tempWorldCoordinates.size(); j++)
            {
                pts[i].rwlp[j].y = -1*tempWorldCoordinates[j].x();
                pts[i].rwlp[j].x = tempWorldCoordinates[j].z();
            }
        }
        tempWorldCoordinates.clear();
        for (int i = 0; i < rndpts; i++)
        {
            // cout << pts[i].x << " " << pts[i].y << " " << pts[i].theta << "\n";
            gradient_descent(pts[i]);
            //levenbergMarquardt(pts[i]);
        }

        sort(pts.begin(), pts.end(), [](const Point &lhs, const Point &rhs) { return lhs.cost < rhs.cost; });

        for (int i = 0; i < 5; i++)
        {
            cout << "Point " << i << " " << pts[i].cost << "\n";
            cout << pts[i].x << " " << pts[i].y << " " << pts[i].theta << "\n";
        }
        cv::Mat image = cv::imread(imagePath);
        std::vector<cv::Point2f> pixelCoordinates = {cv::Point2f(pts[0].x / 24 * image.cols, (16.0 - pts[0].y) / 16 * image.rows), cv::Point2f(pts[1].x / 24 * image.cols, (16.0 - pts[1].y) / 16 * image.rows), cv::Point2f(pts[2].x / 24 * image.cols, (16.0 - pts[2].y) / 16 * image.rows), cv::Point2f(pts[3].x / 24 * image.cols, (16.0 - pts[3].y) / 16 * image.rows), cv::Point2f(pts[4].x / 24 * image.cols, (16.0 - pts[4].y) / 16 * image.rows)};
        std::vector<double> thetas_degrees = {pts[0].theta, pts[1].theta, pts[2].theta, pts[3].theta, pts[4].theta};
        displayPoints(image, pixelCoordinates, thetas_degrees);
    }
}

int odometry_thread()
{
    rclcpp::spin(std::make_shared<OdometrySubscriber>());
    rclcpp::shutdown();
    return 0;
}

int linedetection_thread()
{
    auto camera0_node = std::make_shared<LineSubscriber>("camera","camera_link_optical");
    auto camera1_node = std::make_shared<LineSubscriber>("camera_1","camera_link_optical_1");
    auto camera2_node = std::make_shared<LineSubscriber>("camera_2","camera_link_optical_2");
    auto camera3_node = std::make_shared<LineSubscriber>("camera_3","camera_link_optical_3");

    rclcpp::spin(camera0_node);
    rclcpp::spin(camera1_node);
    rclcpp::spin(camera2_node);
    rclcpp::spin(camera3_node);
    rclcpp::shutdown();
    return 0;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    myfile.open("/home/vihaan/Documents/RoboCup24/localization_ws/src/localization/CamTest.txt");
    odom.x = 0;
    odom.y = 0;
    odom.theta = 0;
    thread t1(odometry_thread);
    thread t3(linedetection_thread);
    thread t2(localization_thread);
    t1.join();
    t3.join();
    t2.join();
    return 0;
}
