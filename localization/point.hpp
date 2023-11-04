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
