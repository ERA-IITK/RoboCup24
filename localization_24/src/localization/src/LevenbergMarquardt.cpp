#include <Eigen/Dense>
#include <iostream>
#include "localization/point.hpp"
#include "localization/linepoints_ambition.hpp"
#include "localization/linepoint.hpp"
#include "localization/realmap_location.hpp"
using namespace Eigen;
typedef Matrix<double, Dynamic, 1> VectorXd;
typedef Matrix<double, Dynamic, Dynamic> MatrixXd;

Point pp;
int maxIterations = 20, c=250;
double lambda = 0.01, epsilon = 1e-5;

double squareError(const vector<WPoint> &givenLinePoints, const vector<WPoint> &estimatedLinePoints)
{
    double sqrerr = 0;
    for (int i = 0; i < (int)estimatedLinePoints.size(); ++i)
    {
        sqrerr += pow(estimatedLinePoints[i].x - givenLinePoints[i].x, 2) + pow(estimatedLinePoints[i].y - givenLinePoints[i].y, 2);
    }
    return sqrerr;
}

double costFunction(const vector<WPoint> &givenLinePoints, const vector<WPoint> &estimatedLinePoints)
{
    double sqrerr = squareError(givenLinePoints, estimatedLinePoints);
    double err = 1 - c * c / (c * c + sqrerr);
    return err;
}

void updateLinePoints()
{
    pp.x = max(1.01, pp.x);
    pp.x = min(22.99, pp.x);
    pp.y = max(1.01, pp.y);
    pp.y = min(14.99, pp.y);
    pp.theta = ((int)pp.theta)%360;

    for (int i = 0; i < (int)pp.rwlp.size(); i++)
    {
        realmap_loc(pp.wlp[i].x, pp.wlp[i].y, pp.rwlp[i].x, pp.rwlp[i].y, pp.x, pp.y, pp.theta);
        WPoint temp(temp.y = min(max(1.01, pp.wlp[i].x), 22.99), temp.x =16.0 - min(max(1.01, pp.wlp[i].y), 14.99));
        linepoint lp = findNearestPoint(temp);
        pp.nlp[i].y = 16.0 - lp.x;
        pp.nlp[i].x = lp.y;
    }
}

vector<double> computeGradients(const vector<double> &parameters, const vector<WPoint> &givenLinePoints, const vector<WPoint> &estimatedLinePoints, const vector<WPoint> &rel_estimatedLinePoints)
{
    vector<double> gradient(parameters.size(), 0.0);
    double err = costFunction(givenLinePoints, estimatedLinePoints);
    double e = squareError(givenLinePoints, estimatedLinePoints);

    double theta = parameters[2] * (M_PI / 180.0);
    double constant = (1 - err) * (1 / (c * c + e));

    gradient[0] = 0;
    gradient[1] = 0;
    gradient[2] = 0;
    for (int i = 0; i < (int)estimatedLinePoints.size(); i++)
    {
        gradient[0] += 2 * constant * (estimatedLinePoints[i].x - givenLinePoints[i].x);
        gradient[1] += 2 * constant * (estimatedLinePoints[i].y - givenLinePoints[i].y);
        gradient[2] += 2 * constant * ((estimatedLinePoints[i].x - givenLinePoints[i].x) * (-1 * rel_estimatedLinePoints[i].x * sin(theta) + -1 * rel_estimatedLinePoints[i].y * cos(theta)) + (estimatedLinePoints[i].y - givenLinePoints[i].y) * (rel_estimatedLinePoints[i].x * cos(theta) - rel_estimatedLinePoints[i].y * sin(theta)));
    }
    return gradient;
}


void f(VectorXd &result)
{
    double temp = costFunction(pp.nlp, pp.wlp);
    result(0) = temp;
}

void J(MatrixXd &result)
{
    vector<double> params = {pp.x, pp.y, pp.theta};
    vector<double> temp = computeGradients(params, pp.nlp, pp.wlp, pp.rwlp);
    result(0, 0) = temp[0];
    result(0, 1) = temp[1];
    result(0, 2) = temp[2];
}

// Levenberg-Marquardt algorithm
void levenbergMarquardt(Point &p)
{
    pp = p;
    updateLinePoints();
    int bit = 1;
    double cost_old = costFunction(pp.nlp, pp.wlp), cost_new, lambda_min = 1e-5, lambda_max = 100;
    std::cout << "Initial Point = " << pp.x << "," << pp.y << "," << pp.theta << std::endl;
    std::cout << "Initial Cost = " << cost_old << std::endl;
    VectorXd residual(1), delta(3);
    MatrixXd Jacobian(1, 3), H(3, 3), Iden = MatrixXd::Identity(Jacobian.cols(), Jacobian.cols());

    for (int iteration = 0; iteration < maxIterations; ++iteration)
    {
        f(residual);
        J(Jacobian);

        if(lambda <= lambda_min)
            lambda = lambda_min;
        if(lambda >= lambda_max)
            lambda = lambda_max;
        H = (Jacobian.transpose() * Jacobian) + lambda * Iden;
        delta = (-H.inverse() * Jacobian.transpose()) * residual;
        pp.x += delta(0);
        pp.y += delta(1);
        pp.theta += delta(2);
        cost_new = costFunction(pp.nlp, pp.wlp);

        // std::cout << "Residual = " << residual(0) << std::endl;
        // std::cout << "Jacobian = " << Jacobian(0, 0) << ", " << Jacobian(0, 1) << ", " << Jacobian(0, 2) << std::endl;
        // std::cout << "Delta = " << delta(0) << ", " << delta(1) << ", " << delta(2) << std::endl;
        // std::cout << "Lambda = " << lambda << std::endl;

        if(cost_new <= cost_old)
        {
            if(bit)
                lambda *= 0.1;
            bit = 1;
            pp.cost = cost_new;
            cost_old = cost_new;
            if (pp.cost < epsilon)
            {
                std::cout << "Localized!" << std::endl;
                break;
            }
        }
        else
        {
            pp.x -= delta(0);
            pp.y -= delta(1);
            pp.theta -= delta(2);
            bit = 0;
            lambda *= 10;
        }
        updateLinePoints();
    }
    std::cout << "Final Point = " << pp.x << "," << pp.y << "," << pp.theta << std::endl;
    std::cout << "Final Cost = " << pp.cost << std::endl;
    p = pp;
}
