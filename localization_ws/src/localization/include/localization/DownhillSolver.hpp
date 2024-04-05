#ifndef DownhillSolver_HPP
#define DownhillSolver_HPP

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
void DownhillSolver(Point &p);
double optimizeFunction(const std::vector<double> &params);

#endif // DownhillSolver_HPP