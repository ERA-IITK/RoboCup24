// point.h

#pragma once  // This ensures that the header is included only once
struct WPoint {
    double x;
    double y;
};

struct Point {
    double x;
    double y;
    double theta;
    vector<WPoint> wlp;
    vector<WPoint> nlp;
};
