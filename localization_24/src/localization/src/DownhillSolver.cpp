#include <iostream>
#include <cmath>
#include <vector>
#include <opencv2/opencv.hpp>
#include "localization/point.hpp"
#include "localization/linepoints_ambition.hpp"
#include "localization/linepoint.hpp"
#include "localization/realmap_location.hpp"
using namespace std;

int numIterations = 20;
int c = 250;
Point pp;
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

void updateLinePoints(Point &p)
{
    for (int i = 0; i < (int)p.rwlp.size(); i++)
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
}

double optimizeFunction(const vector<double> &params)
{
    pp.x = params[0];
    pp.y = params[1];
    pp.theta = params[2];
    double result = costFunction(pp.nlp, pp.wlp);
    return result;
}

void DownhillSolver(Point &p)
{
    pp = p;
    updateLinePoints(pp);
    vector<double> params = {pp.x, pp.y, pp.theta};
    cv::Ptr<cv::DownhillSolver> solver = cv::DownhillSolver::create();
    solver->setFunction(optimizeFunction);
    cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, numIterations, 1e-6);
    solver->setTermCriteria(criteria);
    double result = solver->minimize(params);
    cout<<result<<"\n";
    pp.x = params[0];
    pp.y = params[1];
    pp.theta = params[2];
    updateLinePoints(pp);
}
