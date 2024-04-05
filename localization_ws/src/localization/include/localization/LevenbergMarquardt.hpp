#ifndef LevenbergMarquardt_HPP
#define LevenbergMarquardt_HPP

#include <vector>
#include "point.hpp"
#include "linepoint.hpp"
// Define the structure lp
struct lp {
    double x;
    double y;
};

// Function declarations
double squareError(const std::vector<WPoint> &givenLinePoints, const std::vector<WPoint> &estimatedLinePoints);
double costFunction(const std::vector<WPoint>& estimatedLinePoints, const std::vector<WPoint>& givenLinePoints);
std::vector<double> computeGradients(const std::vector<double>& parameters, const std::vector<WPoint>& givenLinePoints, const std::vector<WPoint>& estimatedLinePoints, const std::vector<WPoint> &rel_estimatedLinePoints);
void levenbergMarquardt(Point &p);
double optimizeFunction(const std::vector<double> &params);

#endif // LevenbergMarquardt_HPP