#ifndef GRADIENT_DESCENT_HPP
#define GRADIENT_DESCENT_HPP

#include <vector>
#include "point.hpp"
#include "linepoint.hpp"
// Define the structure lp
struct lp {
    double x;
    double y;
};

// Function declarations
double costFunction(const std::vector<WPoint>& estimatedLinePoints, const std::vector<WPoint>& givenLinePoints);
void gradientDescentRPROP(std::vector<double>& parameters, std::vector<double>& gradients);
std::vector<double> computeGradients(const std::vector<double>& parameters, const std::vector<WPoint>& givenLinePoints, const std::vector<WPoint>& estimatedLinePoints);
void gradient_descent(Point &p);

#endif // GRADIENT_DESCENT_HPP
