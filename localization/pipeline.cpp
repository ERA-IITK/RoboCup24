#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include "point.hpp"
#include "linepoint.hpp"
#include "linepoints.hpp"
#include "realmap_location.hpp"
#include "utilities.hpp"
#include "rndgeneration.hpp"
#include "raw_points.hpp"
#include "gradient_descent.hpp"

using namespace std;

int main(){
    int rndpts=1000;
    cout<<"Enter number of points: ";
    vector<raw_pt> raw_wlp=rndgen2(5);
    vector<Point> pts=rndgen(1000);
    for(int i=0; i<rndpts; i++){
        pts[i].wlp.resize(raw_wlp.size());
        for(int j=0; j<raw_wlp.size(); j++){
            get_realmap_loc(raw_wlp[j].depth, raw_wlp[j].x_angle, raw_wlp[j].y_angle, pts[i].wlp[j].x, pts[i].wlp[j].y, pts[i].x, pts[i].y, pts[i].theta);
        }
    }
    // for(int i=0; i<rndpts; i++){
    //     for(int j=0; j<raw_wlp.size(); j++){
    //         cout<<pts[i].wlp[j].x<<" "<<pts[i].wlp[j].y<<"\n";
    //     }
    // }

    for(int i=0; i<rndpts; i++){
        pts[i].nlp.resize(raw_wlp.size());
        for(int j=0; j<raw_wlp.size(); j++){
            WPoint k;
            k.x=max(1.01, pts[i].wlp[j].x);
            k.x=min(14.99, k.x);
            k.y=max(1.01, pts[i].wlp[j].y);
            k.y=min(22.99, k.y);
            // cout<<k.x<<" "<<k.y<<"\n";
            linepoint r=findNearestPoint(k);
            pts[i].nlp[j].x=r.x;
            pts[i].nlp[j].y=r.y;
        }
    }
    for(int i=0; i<rndpts; i++){
       gradient_descent(pts[i]);
    }
    sort(pts.begin(), pts.end(), [](const Point& lhs, const Point& rhs) {
        return lhs.cost < rhs.cost;
    });
    pts.resize(100);
    for(int i=0; i<100; i++){
        cout<<pts[i].x<<" "<<pts[i].y<<" "<<pts[i].theta<<"\n";
    }
    return 0;
}