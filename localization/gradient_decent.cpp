#include <iostream>
#include <cmath>
#include <vector>
#include "point.hpp"
#include "linepoints.hpp"

using namespace std;

struct lp{
    double x;
    double y;
};


// Define your cost function (simplified)
double costFunction(const vector<WPoint>& estimatedLinePoints, const vector<WPoint>& givenLinePoints) {
    double cost = 0;
    for (int i = 0; i < estimatedLinePoints.size(); ++i) {
        cost += pow(estimatedLinePoints[i].x - givenLinePoints[i].x, 2)+ pow(estimatedLinePoints[i].y - givenLinePoints[i].y, 2);
    }
    return cost;
}

// Implement the gradient descent with RPROP
void gradientDescentRPROP(vector<double>& parameters, vector<double>& gradients) {
    static vector<double> delta(parameters.size(), 0.1); // Initial update values

    for (int i = 0; i < parameters.size(); ++i) {
        if (gradients[i] * delta[i] > 0) {
            delta[i] = min(delta[i] * 1.2, 50.0);
        }
        else if (gradients[i] * delta[i] < 0) {
            delta[i] = max(delta[i] * 0.5, 1e-6);
            gradients[i] = 0;
        }

        parameters[i] -= copysign(delta[i], gradients[i]);
    }
}

// Placeholder for the computeGradients function
vector<double> computeGradients(const vector<double>& parameters, const vector<WPoint>& givenLinePoints, const vector<WPoint>& estimatedLinePoints) {
    // Implement your gradient calculation logic here based on your specific problem
    // This function should return the gradient vector
    vector<double> gradient(parameters.size(), 0.0);
    double c = 250;
    double e=0;
    for (int i = 0; i < estimatedLinePoints.size(); ++i) {
        e += pow(estimatedLinePoints[i].x - givenLinePoints[i].x, 2)+ pow(estimatedLinePoints[i].y - givenLinePoints[i].y, 2);
    }

    double err = -c*c/pow((c*c+e*e),2);

    // Placeholder example: Setting gradients to 0 for demonstration
    // Replace this with your actual gradient computation
    double x = parameters[0];
    double y = parameters[1];
    double theta = parameters[2];

    gradient[0]=0;
    gradient[1]=0;
    gradient[2]=0;
    for(int i=0;i<estimatedLinePoints.size();i++){
        gradient[0] += 2*(givenLinePoints[i].x-estimatedLinePoints[i].x)*err;
        gradient[1] += 2*(givenLinePoints[i].y-estimatedLinePoints[i].y)*err;
        gradient[2] += 2*((x-estimatedLinePoints[i].x)*(givenLinePoints[i].x*sin(theta)+givenLinePoints[i].y*cos(theta)) + (y-estimatedLinePoints[i].y)*(givenLinePoints[i].x*cos(theta)-givenLinePoints[i].y*sin(theta)))*err;
    }


    return gradient;
}

void gradient_descent(Point &p) {
    // Initialize your initial estimates for x, y, and theta
    vector<double> parameters = {p.x,p.y,p.theta};

    // Your given line points and estimated line points
    vector<WPoint> givenLinePoints = p.wlp;
    vector<WPoint> estimatedLinePoints;

    // Number of iterations
    int numIterations = 1000;

    for (int iteration = 0; iteration < numIterations; ++iteration) {
        // Compute gradients based on your cost function
        if(iteration==0){
            for(int i=0;i<givenLinePoints.size();i++){
            Point temp ;
            temp.x=givenLinePoints[i].x;
            temp.y=givenLinePoints[i].y;
            linepoint lp = findNearestPoint(temp);
            estimatedLinePoints[i].x = lp.x;
            estimatedLinePoints[i].y = lp.y;
        }

        }
        else for(int i=0;i<estimatedLinePoints.size();i++){
            Point temp ;
            temp.x=estimatedLinePoints[i].x;
            temp.y=estimatedLinePoints[i].y;
            linepoint lp = findNearestPoint(temp);
            estimatedLinePoints[i].x = lp.x;
            estimatedLinePoints[i].y = lp.y;
        }

        vector<double> gradients = computeGradients(parameters, givenLinePoints, estimatedLinePoints);

        // Update parameters using RPROP
        gradientDescentRPROP(parameters, gradients);

        // Calculate the cost for this iteration
        double cost = costFunction(estimatedLinePoints, givenLinePoints);
        cout << "Iteration " << iteration << ": Cost = " << cost << endl;
    }

    // Output the final estimated parameters
    cout << "Final Parameters (x, y, theta): ";
    for (double param : parameters) {
        cout << param << " ";
    }
    cout << endl;
}