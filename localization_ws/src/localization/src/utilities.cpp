#include <vector>
#include <cmath>
#include "../include/localization/point.hpp"
#include "../include/localization/utilities.hpp"
using namespace std;
double mse(double x1, double x2, double y1, double y2){
    double error = pow(x1 - x2, 2) + pow(y1 - y2, 2);
    return error;
}
vector<double> computeMSELoss(vector<Point> p) {
    vector<double> meanLoss(p.size(), 0.0);
    int j=0;
    for(auto whiteLinePoints : p){
        double totalLoss=0.0;
        for (int i = 0; i < whiteLinePoints.wlp.size(); i++) {
            totalLoss+= mse(whiteLinePoints.wlp[i].x, whiteLinePoints.nlp[i].x, whiteLinePoints.wlp[i].y, whiteLinePoints.nlp[i].y);
        }
        meanLoss[j] = totalLoss / whiteLinePoints.wlp.size();
        j++;
    }
    
    return meanLoss;
}

void addscore(vector<Point> p){
    p[0].score++;
}

void inc_age(vector<Point> p){
    for(auto pt: p){
        pt.age++;
    }
}

void del_nodes(vector<Point>& p, int threshold) { // threshold to be determined experimentally
    auto it = p.begin();
    while (it != p.end()) {
        if (it->age - it->score >= threshold) {
            it = p.erase(it);
        } else {
            ++it;
        }
    }
}

// update odometry of points
void update_odom(vector<Point>& p, odometry odom) {
    for (auto& pt : p) {
        pt.x += odom.x;
        pt.y += odom.y;
        pt.theta += odom.theta;
    }
}