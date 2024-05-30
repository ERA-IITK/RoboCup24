#include "localization/linepoints_ambition.hpp"
using namespace std;

// origin is at bottom left corner of field when field is viewed in landscape mode

bool CompareByD(const linepoint &a, const linepoint &b)
{
    // cout<<a.d<<" "<<b.d<<endl;
    return a.d < b.d;
}

float distance(float x1, float y1, float x2, float y2)
{
    return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}
// uncomment to test the code
float randx()
{
    float minv = 1;
    float maxv = 15;
    int num = static_cast<int>((minv - maxv) * 10) + 1;
    int random_index = rand() % num;
    float random_float_value = minv + random_index * 0.1;
    return random_float_value;
}

float randy()
{
    float minv = 1;
    float maxv = 11;
    int num = static_cast<int>((minv - maxv) * 10) + 1;
    int random_index = rand() % num;
    float random_float_value = minv + random_index * 0.1;
    return random_float_value;
}

int rantheta()
{
    int min_value = 0;   // Minimum value
    int max_value = 360; // Maximum value
    int random_integer = rand() % (max_value - min_value + 1) + min_value;
    return random_integer;
}

/*********************************The locuses of all lines on the field*********************************/

/*
the left most line                      :    x=1                                   (1<=y<=23)
the right most line                     :    x=15                                  (1<=y<=23)
the bottom most line                    :    y=1                                   (1<=x<=15)
the top most line                       :    y=23                                  (1<=x<=15)
the middle line                         :    y=12                                  (1<=x<=15)
outward rectangles at goal              :    y=0.25                                (6.8<=x<=9.2)            [BOTTOM]                     horizontal
(__do__)                                :    y=23.75                               (6.8<=x<=9.2)            [TOP]                        horizontal
(__do__)                                :    x=6.8                                 (0.25<=y<=1)    &&  (23<=y<=23.75)                    left vertical
(__do__)                                :    x=9.2                                 (0.25<=y<=1)    &&  (23<=y<=23.75)                    right vertical
inner rectangles at centre (small)      :    y=1.75                                (6.05<=x<=9.95)          [BOTTOM]                     horizontal
(__do__)                                :    y=22.25                               (6.05<=x<=9.95)          [TOP]                        horizontal
(__do__)                                :    x=6.05                                (1<=y<=1.75)    &&  (22.25<=y<=23)                    left vertical
(__do__)                                :    x=9.95                                (1<=y<=1.75)    &&  (22.25<=y<=23)                    right vertical
inner rectangle at cemtre (big)         :    y=20.75                               (4.55<=x<=11.45)         [BOTTOM]                     horizontal
(__do__)                                :    y=3.25                                (4.55<=x<=11.45)         [TOP]                        horizontal
(__do__)                                :    x=4.55                                (1<=y<=3.25)    &&  (20.75<=y<=23)                    left vertical
(__do__)                                :    x=11.45                               (1<=y<=3.25)    &&  (20.75<=y<=23)                    right vertical
circle at centre                        :    x^2 + y^2 - 16x - 24y + 204 = 0       (6<=x<=10)      &&  (10<=y<=14)
quadrant at bottom left corner          :    x^2 + y^2 - 2x - 2y + 1.4375 = 0      (1<=x<=1.75)    &&  (1<=y<=1.75)
quadrant at bottom right corner         :    x^2 + y^2 - 30x - 2y + 225.4375 = 0   (14.25<=x<=15)  &&  (1<=y<=1.75)
quadrant at top left corner             :    x^2 + y^2 - 2x - 46y + 529.4375 = 0   (1<=x<=1.75)    &&  (22.25<=y<=23)
quadrant at top right corner            :    x^2 + y^2 - 30x - 46y + 753.4375 = 0  (14.25<=x<=15)  &&  (22.25<=y<=23)
*/

const vector<line> lines = {
    {0, 1, 1},
    {0, 1, 11},
    {1, 0, 1},
    {1, 0, 15},//outer

    // {0, 1, 12},//y bich
    {1, 0, 0.25},
    {1, 0, 15.75},
    {0, 1, 4.8},
    {0, 1, 7.2},//goal outward
    {0, 1, 4.8},
    {0, 1, 7.2},//rep for 2

    {1, 0, 1.75},
    {1, 0, 14.25},
    {0, 1, 4.05},
    {0, 1, 7.95},//chota d
    {0, 1, 4.05},
    {0, 1, 7.95},//rep for 2
    // {0, 1, 20.75},
    // {0, 1, 3.25},
    // {1, 0, 4.55},
    // {1, 0, 11.45},

    {0, 1, 1.75},

    {1, 0, 6.05},
    {1, 0, 9.95},//chota d

    {0, 1, 3.25},
    {1, 0, 4.55},
    {1, 0, 11.45},//bada d
};

const vector<limits> Lim = {
    {1, 15 , 1, 1},
    {1, 15, 11, 11},
    {1, 1, 1, 11},
    {15, 15, 1, 11},

    // {1, 15, 12, 12},
    {0.25, 0.25, 4.8, 7.2},
    {15.75, 15.75, 4.8, 7.2},
    {0.25, 1, 4.8, 4.8}, //*
    {0.25, 1, 7.2, 7.2}, //*

    {15, 15.75, 4.8, 4.8}, //*
    {15, 15.75, 7.2, 7.2}, //*

    {1.75, 1.75, 4.05, 7.95},
    {14.25, 14.25, 4.05, 7.95},
    {1, 1.75, 4.05, 4.05}, //*
    {1, 1.75, 7.95, 7.95}, //*

    {14.25, 15, 4.05, 4.05}, //*
    {14.25, 15, 7.95, 7.95},
    // {4.55, 11.45, 20.75, 20.75},
    // {4.55, 11.45, 3.25, 3.25},
    // {4.55, 4.55, 1, 3.25},   //*
    // {11.45, 11.45, 1, 3.25}, //*

    {6.05, 9.95, 1.75, 1.75},

    {6.05, 6.05, 1, 1.75}, //*
    {9.95, 9.95, 1, 1.75}, //*

    {4.55, 11.45, 3.25, 3.25},
    {4.55, 4.55, 1, 3.25},   //*
    {11.45, 11.45, 1, 3.25}, //*
};

linepoint findNearestPoint(WPoint a)
{
    vector<linepoint> all_points(lines.size(),{0,0,0});
    double x = a.x;
    double y = a.y;
    for (int i = 0; i < all_points.size(); i++){
        double a = lines[i].a;
        double b = lines[i].b;
        double c = lines[i].c;

        if(a==0){
            all_points[i].y = c;
            if(x<Lim[i].xmin) all_points[i].x = Lim[i].xmin;
            else if(x>Lim[i].xmax) all_points[i].x = Lim[i].xmax;
            else all_points[i].x=x;
        }
        else if (b==0)
        {
            all_points[i].x = c;
            if(y<Lim[i].ymin) all_points[i].y = Lim[i].ymin;
            else if(y>Lim[i].ymax) all_points[i].y = Lim[i].ymax;
            else all_points[i].y=y;
        }
    }
    linepoint corner;
    corner = {1,1,0};
    // all_points.push_back(corner);
    // corner = {15,11,0};
    // all_points.push_back(corner);
    // corner = {15,1,0};
    // all_points.push_back(corner);
    // corner = {1,11,0};
    // all_points.push_back(corner);

    double theta;
    theta = atan((y - 1) / (x - 1));
    corner.x = 1 + 0.75 * cos(theta);
    corner.y = 1 + 0.75 * sin(theta);
    all_points.push_back(corner);

    theta = atan((y - 1) / (15 - x));
    corner.x = 15 - 0.75 * cos(theta);
    corner.y = 1 + 0.75 * sin(theta);
    all_points.push_back(corner);

    for (int i = 0; i < all_points.size(); i++){
        all_points[i].d = distance(x, y, all_points[i].x, all_points[i].y);
    }
    // for (int i = 0; i < all_points.size(); i++)
    // {
    //     cout<<all_points[i].x<<" ";
    //     cout<<all_points[i].y<<" ";
    //     cout<<all_points[i].d<<" ";
    //     cout<<endl;
    // }
    sort(all_points.begin(), all_points.end(), CompareByD);
    
    
    linepoint p = all_points[0];
    all_points.clear();
    return p;
}

// // uncomment to test the code
// int main() {
//      srand(static_cast<unsigned>(time(nullptr)));

//     WPoint a;
//     a.x = randx();
//     a.y = randy();

//     linepoint nearestPoint = findNearestPoint(a);

//     cout << "Given Point: {" << a.x << ", " << a.y << "}" << endl;
//     cout << "Nearest Point: {" << nearestPoint.x << ", " << nearestPoint.y << ", " << nearestPoint.d << "}" << endl;

//     return 0;
// }
