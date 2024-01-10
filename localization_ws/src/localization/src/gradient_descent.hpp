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
double squareError(const vector<WPoint> &givenLinePoints, const vector<WPoint> &estimatedLinePoints);
double costFunction(const std::vector<WPoint>& estimatedLinePoints, const std::vector<WPoint>& givenLinePoints);
void adamGradientDescent(std::vector<double> &parameters, const std::vector<double> &gradients, std::vector<double> &m, std::vector<double> &v, int t, double learning_rate = 0.5, double beta1 = 0.9, double beta2 = 0.999, double epsilon = 1e-8);
std::vector<double> computeGradients(const std::vector<double>& parameters, const std::vector<WPoint>& givenLinePoints, const std::vector<WPoint>& estimatedLinePoints, const std::vector<WPoint> &rel_estimatedLinePoints);
void gradient_descent(Point &p);

#endif // GRADIENT_DESCENT_HPP
