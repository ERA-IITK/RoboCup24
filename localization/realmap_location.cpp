#include <cmath>

// Define camera parameters
const double camera_fov = 110.0; // Field of view in degrees
const double camera_width = 640.0; // Image width in pixels
const double camera_height = 360.0; // Image height in pixels

// Define function to get projection of point in 2D plane
void get_projection(double depth, double angle_x, double angle_y, double& x, double& y) {
    // Convert angles to radians
    double rad_x = angle_x * M_PI / 180.0;
    double rad_y = angle_y * M_PI / 180.0;

    // Calculate distance from camera to point
    double distance = depth / cos(rad_x);

    // Calculate x and y coordinates of point in camera frame
    double x_cam = distance * tan(rad_y);
    double y_cam = distance * tan(rad_x);

    // Calculate x and y coordinates of point in image frame
    x = (x_cam / camera_width + 0.5) * camera_width;
    y = (y_cam / camera_height + 0.5) * camera_height;
}

// Define function to get projection of point in 2D plane
void get_realmap_loc(double depth, double angle_x, double angle_y, double& x, double& y, double x_global, double y_global, double theta) {
    get_projection(depth, angle_x, angle_y, x, y);
    // global map coordinates
    double the=theta*M_PI/180.0;
    realmap_loc(x, y, x_global, y_global, the);
}

void realmap_loc(double& x, double& y, double x_global, double y_global, double the){
    x=x_global+x*cos(the)-y*sin(the);
    y=y_global+x*sin(the)+y*cos(the);
}