#include <iostream>
#include <cmath>
#include <vector>
#include "localization/point.hpp"
#include "localization/linepoints.hpp"
#include "localization/linepoint.hpp"
#include "localization/realmap_location.hpp"
using namespace std;

int c = 250;
int numIterations = 20;
double learningRate = 0.5;
double beta1 = 0.75;
double beta2 = 0.88;

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

void adamGradientDescent(vector<double> &parameters, const vector<double> &gradients, vector<double> &m, vector<double> &v, int t, double learning_rate = 0.5, double beta1 = 0.9, double beta2 = 0.999, double epsilon = 1e-8)
{
    // Hyperparameters:
    // learning_rate: Step size for gradient descent.
    // beta1: Exponential decay rate for the first moment estimate.
    // beta2: Exponential decay rate for the second moment estimate.
    // epsilon: Small value to prevent division by zero.

    const double one_minus_beta1 = 1.0 - beta1;
    const double one_minus_beta2 = 1.0 - beta2;
    const int num_parameters = parameters.size();

    for (int i = 0; i < num_parameters; ++i)
    {
        m[i] = beta1 * m[i] + one_minus_beta1 * gradients[i];
        v[i] = beta2 * v[i] + one_minus_beta2 * (gradients[i] * gradients[i]);
        double m_hat = m[i] / (1.0 - pow(beta1, t));
        double v_hat = v[i] / (1.0 - pow(beta2, t));
        parameters[i] -= learning_rate * m_hat / (sqrt(v_hat) + epsilon);
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

    // cout << "Grads " << gradient[0] << ", " << gradient[1] << ", " << gradient[2] << "\n";
    return gradient;
}

void updateLinePoints(Point &p)
{
    p.x = max(1.01, p.x);
    p.x = min(22.99, p.x);
    p.y = max(1.01, p.y);
    p.y = min(14.99, p.y);
    p.theta = ((int)p.theta)%360;

    for (int i = 0; i < (int)p.rwlp.size(); i++)
    {
        realmap_loc(p.wlp[i].x, p.wlp[i].y, p.rwlp[i].x, p.rwlp[i].y, p.x, p.y, p.theta);
        WPoint temp;
        temp.y = min(max(1.01, p.wlp[i].x), 22.99);
        temp.x =16.0-(min(max(1.01, p.wlp[i].y), 14.99));
        // cout << temp.x << ", " << temp.y << " temp\n";
        linepoint lp = findNearestPoint(temp);
        // cout << lp.x << ", " << lp.y << " fnp\n";
        p.nlp[i].y = 16.0 - lp.x;
        p.nlp[i].x = lp.y;
        // cout << p.wlp[i].x << ", " << p.wlp[i].y << " is " << p.nlp[i].x << ", " << p.nlp[i].y << "\n";
    }
}

void gradient_descent(Point &p)
{
    vector<double> parameters = {p.x, p.y, p.theta};
    vector<double> m(parameters.size(), 0.0);
    vector<double> v(parameters.size(), 0.0);

    for (int iteration = 0; iteration < numIterations; ++iteration)
    {
        updateLinePoints(p);
        // if (iteration == 0)
        //     cout << "Initial Cost " << costFunction(p.nlp, p.wlp) << " (" << p.x << ", " << p.y << ", " << p.theta << ")" << endl;
        // else
        //     cout << "Iteration " << iteration << ": SqErr = " << squareError(p.nlp, p.wlp) << " (" << p.x << ", " << p.y << ", " << p.theta << ")" << endl;

        vector<double> gradients = computeGradients(parameters, p.nlp, p.wlp, p.rwlp);
        adamGradientDescent(parameters, gradients, m, v, iteration + 1, learningRate, beta1, beta2);

        p.x = parameters[0];
        p.y = parameters[1];
        p.theta = parameters[2];
        p.cost = costFunction(p.nlp, p.wlp);
    }

    updateLinePoints(p);
    // cout << "Iteration " << numIterations << ": SqErr = " << squareError(p.nlp, p.wlp) << endl;
    // cout << "(" << p.x << ", " << p.y << ", " << p.theta << ")" << endl;
    // cout << "Final Cost " << costFunction(p.nlp, p.wlp) << endl;
    // cout << endl;
}