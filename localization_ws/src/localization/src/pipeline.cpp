#include <iostream>
#include <fstream>
#include <vector>
#include <deque>
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
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <chrono>

using std::placeholders::_1;
using namespace std;
odometry odom;
vector<Eigen::Vector3d> worldCoordinates;
deque<vector<Eigen::Vector3d>> worldCoordinatesDeque;
ofstream myfile;
int counter = 0;
float cx, cy, fx, fy;

void displayPoints(cv::Mat& image, const std::vector<cv::Point2f>& points, const std::vector<double>& thetas_degrees) {
    // Draw arrows for each point on the image
    for (size_t i = 0; i < points.size(); ++i) {
        const auto& point = points[i];
        const double theta_degrees = thetas_degrees[i];
        const double theta_radians = cv::fastAtan2(-std::sin(cv::fastAtan2(0.0, -1.0) * (theta_degrees / 180.0)),
                                                  std::cos(cv::fastAtan2(0.0, -1.0) * (theta_degrees / 180.0)));

        // Set color based on index (blue for the first point, yellow for the rest)
        cv::Scalar color = (i == 0) ? cv::Scalar(255, 0, 0) : cv::Scalar(0, 255, 255);

        // Draw circle
        cv::circle(image, point, 5, color, -1);  // Circle radius: 5

        // Draw arrow representing the facing direction
        int arrowLength = 30;
        cv::Point2f arrowEnd(static_cast<float>(point.x + arrowLength * std::cos(theta_radians)),
                              static_cast<float>(point.y + arrowLength * std::sin(theta_radians)));
        cv::arrowedLine(image, point, arrowEnd, color, 2);
    }

    // Display the image with points and arrows
    cv::imshow("Image with Points", image);

    cv::waitKey(1000);  // Adjust waitKey delay (1 ms here) as needed for real-time updates
}

class OdometrySubscriber : public rclcpp::Node
{
public:
    OdometrySubscriber() : Node("odom_subscriber_node")
    {
        // Subscribe to the /odom topic
        subscription_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&OdometrySubscriber::odomCallback, this, std::placeholders::_1));
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Callback function for /odom topic
        // RCLCPP_INFO(get_logger(), "I heard : '%f %f'", msg->pose.pose.position.x, msg->pose.pose.position.y);

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
        // RCLCPP_INFO(rclcpp::get_logger("odom_listener"), "Yaw angle with respect to y-axis: %f degrees", yaw_degrees);
        odom.x = msg->pose.pose.position.x;
        odom.y = msg->pose.pose.position.y;
        odom.theta = yaw_degrees;
    }
};
class LineSubscriber : public rclcpp::Node
{
public:
    LineSubscriber() : Node("my_subscriber_node")
    {
        image_subscriber_ = create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw",
            10, // Set the queue size
            std::bind(&LineSubscriber::imageCallback, this, std::placeholders::_1));

        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/depth/image_raw", 10, std::bind(&LineSubscriber::depth_callback, this, std::placeholders::_1));

        // Subscribe to the camera_info topic
        camera_info_subscriber_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera/camera_info",
            10, // Set the queue size
            std::bind(&LineSubscriber::cameraInfoCallback, this, std::placeholders::_1));
    }

private:
    Eigen::Matrix3d cameraMatrix;
    std::vector<std::vector<cv::Point>> contours;

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
        cx = arrayData[2];
        cy = arrayData[5];
        fx = arrayData[0];
        fy = arrayData[4];
    }

    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
        contours = linedetection(cv_ptr->image);
    }

    void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        counter++;
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

                // float u = (point.x - cx)/fx;
                // float v = (point.y - cy)/fy;
                // float xx = u * depth_value;
                // float yy = v * depth_value;
                // float zz = depth_value;
                // Eigen::Vector3d worldCoords;
                // worldCoords << xx, yy, zz;

                // Store the result
                float beta = 1.3907;
                Eigen::Matrix3d rotationMatrix;
                rotationMatrix << cos(beta), 0, sin(beta), 0, 1, 0, -sin(beta), 0, cos(beta);
                // worldCoords = rotationMatrix * worldCoords;

                worldCoords.x() *= depth_value;
                worldCoords.y() *= depth_value;
                worldCoords.z() *= depth_value;

                myfile << "(" << point.x << ", " << point.y << ") is (" << worldCoords.x() << "," << worldCoords.y() << "," << worldCoords.z() << ")\n";

                if (counter >= 5)
                    myfile.close();

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

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
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
    vector<Point> pts = rndgen(rndpts);
    odometry temp;
    temp.x = 0, temp.y = 0, temp.theta = 0;
    int k = 0;
    std::string imagePath = "/home/suryansh/Documents/GitHub/RoboCup24/localization_ws/src/localization/images/robocup_field.png";
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
        inc_age(pts);
        addscore(pts);
        del_nodes(pts);

        for (int i = 0; i < rndpts; i++)
        {
            pts[i].rwlp.resize(tempWorldCoordinates.size());
            pts[i].wlp.resize(tempWorldCoordinates.size());
            pts[i].nlp.resize(tempWorldCoordinates.size());

            for (int j = 0; j < tempWorldCoordinates.size(); j++)
            {
                pts[i].rwlp[j].x = tempWorldCoordinates[j].z();
                pts[i].rwlp[j].y = tempWorldCoordinates[j].x();
            }
        }
        tempWorldCoordinates.clear();
        for (int i = 0; i < rndpts; i++)
        {
            // cout << pts[i].x << " " << pts[i].y << " " << pts[i].theta << "\n";
            gradient_descent(pts[i]);
        }

        sort(pts.begin(), pts.end(), [](const Point &lhs, const Point &rhs)
             { return lhs.cost < rhs.cost; });

        for (int i = 0; i < 5; i++)
        {
            cout << pts[i].x << " " << pts[i].y << " " << pts[i].theta << "\n";
            cout << "Point " << i << " " << pts[i].cost << "\n";
        }
        cv::Mat image = cv::imread(imagePath);
        std::vector<cv::Point2f> pixelCoordinates = {cv::Point2f(pts[0].y/24*image.cols, pts[0].x/16*image.rows), cv::Point2f(pts[1].y/24*image.cols, pts[1].x/16*image.rows), cv::Point2f(pts[2].y/24*image.cols, pts[2].x/16*image.rows), cv::Point2f(pts[3].y/24*image.cols, pts[3].x/16*image.rows), cv::Point2f(pts[4].y/24*image.cols, pts[4].x/16*image.rows)};
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
    rclcpp::spin(std::make_shared<LineSubscriber>());
    rclcpp::shutdown();
    return 0;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    myfile.open("/home/legendarygene/Desktop/RoboCup24/localization_ws/src/localization/CamTest.txt");
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