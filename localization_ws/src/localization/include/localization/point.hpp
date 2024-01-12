// point.hpp
#pragma once

#include <vector>

using namespace std;
class WPoint {
public:
    double x=0;
    double y=0;

    // Constructor
    WPoint(): x{0.0}, y{0.0} {}
    WPoint(double _x, double _y) : x(_x), y(_y) {}
};

class Point {
public:
    double x;
    double y;
    double theta;
    double cost=1e8;
    double score=0;
    double age=0;
    vector<WPoint> wlp;
    vector<WPoint> nlp;
    vector<WPoint> rwlp;
};

class odometry {
public:
    double x;
    double y;
    double theta;
    odometry(): x{0.0}, y{0.0}, theta{0.0} {}
    odometry(double _x, double _y, double _theta) : x(_x), y(_y), theta(_theta) {}
};

int main() {
    odometry odom;
    odom.x = 0;
    odom.y = 0;
    odom.theta = 0;

    // Rest of your code...

    return 0;
}