#include <vector>
#include <cmath>
#include "point.hpp"
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
    vector<double> meanLoss=computeMSELoss(p);
    int mx=-1, ind=-1;
    for(int i=0; meanLoss.size(); i++){
        if(meanLoss[i]>mx){
            mx=meanLoss[i];
            ind=i;
        }
    }
    p[ind].score++;
}


