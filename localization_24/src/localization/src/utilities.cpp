#include <vector>
#include <cmath>
#include "localization/point.hpp"
#include "localization/utilities.hpp"
using namespace std;

void addscore(vector<Point> p)
{
    p[0].score++;
}

void inc_age(vector<Point> p)
{
    for(auto pt: p)
    {
        pt.age++;
    }
}

void del_nodes(vector<Point>& p, int threshold)
{ // threshold to be determined experimentally
    auto it = p.begin();
    while (it != p.end())
    {
        if (it->age - it->score >= threshold)
        {
            it = p.erase(it);
        } 
        else
        {
            ++it;
        }
    }
}

// update odometry of points
void update_odom(vector<Point>& p, odometry odom)
{
    for (auto& pt : p)
    {
        pt.x += odom.x;
        pt.y += odom.y;
        pt.theta += odom.theta;
    }
}