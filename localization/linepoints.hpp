#include <vector>
#include "point.hpp"

struct linepoint {
    float x;
    float y;
    float d;
};

linepoint findNearestPoint(Point a);
std::vector<linepoint> linepoints(Point a);