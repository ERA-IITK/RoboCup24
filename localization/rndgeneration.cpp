#include <iostream>
#include <random>
#include <vector>
#include "point.h"  // Include the header file containing the Point struct

using namespace std;

// Define the rndgen function to generate random points
vector<Point> rndgen(int num_points) {
    random_device rd;
    mt19937 gen(rd());

    uniform_real_distribution<double> x_dist(0.0, 16.0);
    uniform_real_distribution<double> y_dist(0.0, 24.0);
    uniform_real_distribution<double> theta_dist(0.0, 360.0);

    vector<Point> random_points;

    for (int i = 0; i < num_points; ++i) {
        Point point;
        point.x = x_dist(gen);
        point.y = y_dist(gen);
        point.theta = theta_dist(gen);
        random_points.push_back(point);
    }

    return random_points;
}
// Uncomment to test the code
int main() {
    // Call the rndgen function to generate random points
    vector<Point> points = rndgen(1000);

    // You can access and print the generated points if needed
    for (const auto& point : points) {
        cout << "(" << point.x << ", " << point.y << ", " << point.theta << ")\n";
    }
    return 0;
}
