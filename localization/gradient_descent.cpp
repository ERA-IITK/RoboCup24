#include <iostream>
#include <cmath>
#include <vector>
#include "point.hpp"
#include "linepoints.hpp"
#include "linepoint.hpp"
#include "realmap_location.hpp"

using namespace std;
int c = 50;
int numIterations = 10;

double mse(const vector<WPoint> &givenLinePoints, const vector<WPoint> &estimatedLinePoints)
{
    double cost = 0;
    for (int i = 0; i < estimatedLinePoints.size(); ++i)
    {
        cost += pow(givenLinePoints[i].x - estimatedLinePoints[i].x, 2) + pow(givenLinePoints[i].y - estimatedLinePoints[i].y, 2);
    }
    return cost;
}

double costFunction(const vector<WPoint> &givenLinePoints, const vector<WPoint> &estimatedLinePoints)
{
    double cost = mse(givenLinePoints, estimatedLinePoints);
    double err = 1 - c * c / pow((c * c + cost * cost), 2);
    return err;
}

void gradientDescentRPROP(vector<double> &parameters, vector<double> &gradients, vector<double> &delta)
{
    for (int i = 0; i < parameters.size(); ++i)
    {
        if (gradients[i] * delta[i] > 0)
        {
            delta[i] = min(delta[i] * 1.2, 50.0);
        }
        else if (gradients[i] * delta[i] < 0)
        {
            delta[i] = max(delta[i] * 0.5, 1e-4);
            gradients[i] = 0;
        }
        parameters[i] -= copysign(delta[i], gradients[i]);
    }
}

vector<double> computeGradients(const vector<double> &parameters, const vector<WPoint> &givenLinePoints, const vector<WPoint> &estimatedLinePoints, const vector<WPoint> &rel_estimatedLinePoints)
{
    vector<double> gradient(parameters.size(), 0.0);
    double err = costFunction(givenLinePoints, estimatedLinePoints);
    double e = mse(givenLinePoints, estimatedLinePoints);

    double x = parameters[0];
    double y = parameters[1];
    double theta = parameters[2] * (M_PI / 180.0);
    double constant = (4 * e / c) * (pow(sqrt(1 - err), 3));

    gradient[0] = 0;
    gradient[1] = 0;
    gradient[2] = 0;
    for (int i = 0; i < estimatedLinePoints.size(); i++)
    {
        gradient[0] += 2 * constant * (givenLinePoints[i].x - estimatedLinePoints[i].x);
        gradient[1] += 2 * constant * (givenLinePoints[i].y - estimatedLinePoints[i].y);
        gradient[2] += 2 * constant * ((givenLinePoints[i].x - estimatedLinePoints[i].x) * (givenLinePoints[i].x - x + rel_estimatedLinePoints[i].x * sin(theta) + rel_estimatedLinePoints[i].y * cos(theta)) + (givenLinePoints[i].y - estimatedLinePoints[i].y) * (givenLinePoints[i].y - y - rel_estimatedLinePoints[i].x * cos(theta) + rel_estimatedLinePoints[i].y * sin(theta)));
    }

    return gradient;
}

void gradient_descent(Point &p)
{
    vector<double> parameters = {p.x, p.y, p.theta};
    vector<double> delta(parameters.size(), 0.1);

    for (int iteration = 0; iteration < numIterations; ++iteration)
    {
        for (int i = 0; i < p.nlp.size(); i++)
        {
            realmap_loc(p.wlp[i].x, p.wlp[i].y, p.rwlp[i].x, p.rwlp[i].y, p.x, p.y, p.theta);
            WPoint temp(temp.x = p.wlp[i].x, temp.y = p.wlp[i].y);
            temp.x = max(1.01, p.wlp[i].x);
            temp.x = min(14.99, temp.x);
            temp.y = max(1.01, p.wlp[i].y);
            temp.y = min(22.99, temp.y);
            linepoint lp = findNearestPoint(temp);
            p.nlp[i].x = lp.x;
            p.nlp[i].y = lp.y;
        }
        if (iteration == 0)
        {
            double tempo = mse(p.nlp, p.wlp);
            cout << "Initial Cost  = " << tempo << endl;
        }

        vector<double> gradients = computeGradients(parameters, p.nlp, p.wlp, p.rwlp);
        gradientDescentRPROP(parameters, gradients, delta);
        double cost = mse(p.nlp, p.wlp);

        p.x = parameters[0];
        p.y = parameters[1];
        p.theta = parameters[2];
        p.cost = cost;

        cout << "Iteration " << iteration + 1 << ": Cost = " << cost << endl;
    }

    cout << "Final Parameters (x, y, theta): ";
    for (double param : parameters)
    {
        cout << param << " ";
    }
    cout << endl;
}
