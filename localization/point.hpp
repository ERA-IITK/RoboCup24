// point.hpp
#pragma once

#include <vector>
using namespace std;
class WPoint {
public:
    double x;
    double y;

    // Constructor
    WPoint(double _x, double _y) : x(_x), y(_y) {}
};

class Point {
public:
    double x;
    double y;
    double theta;
    double score=0;
    double age=0;
    vector<WPoint> wlp;
    vector<WPoint> nlp;
};
