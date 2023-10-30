#include <cmath>
#include "realmap_location.hpp"

// Define function to get projection of point in 2D plane
void get_projection(double depth, double x_angle, double y_angle, double &x, double &y)
{
    // Convert angles to radians
    double rad_x = x_angle * M_PI / 180.0;
    double rad_y = y_angle * M_PI / 180.0;

    // Calculate distance from camera to point
    double distance = depth / cos(rad_x);

    // Calculate x and y coordinates of point in camera frame
    double x_cam = distance * tan(rad_y);
    double y_cam = distance * tan(rad_x);

    // // Calculate x and y coordinates of point in image frame
    // x = (x_cam / camera_width + 0.5) * camera_width;
    // y = (y_cam / camera_height + 0.5) * camera_height;
    x = x_cam;
    y = y_cam;
}

// Define function to get projection of point in 2D plane
void get_realmap_loc(double depth, double x_angle, double y_angle, double &x, double &y, double x_global, double y_global, double theta)
{
    get_projection(depth, x_angle, y_angle, x, y);
    // global map coordinates
    double the = theta * M_PI / 180.0;
    realmap_loc(x, y, x, y, x_global, y_global, the);
}

void realmap_loc(double &x, double &y, double &rel_x, double &rel_y, double x_global, double y_global, double the)
{
    the *= M_PI / 180.0;
    x = x_global + rel_x * cos(the) - rel_y * sin(the);
    y = y_global + rel_x * sin(the) + rel_y * cos(the);
}