#include "linepoints.hpp"
using namespace std;

//origin is at bottom left corner of field when field is viewed in landscape mode

bool CompareByD(const linepoint &a, const linepoint &b){
    // cout<<a.d<<" "<<b.d<<endl;
    return a.d < b.d;
}

float distance(float x1,float y1,float x2,float y2){
    return sqrt(pow(x1-x2,2)+pow(y1-y2,2));
}
// uncomment to test the code
float randx(){
    float minv = 1;
    float maxv = 15;
    int num = static_cast<int>((minv-maxv)*10) +1;
    int random_index = rand() % num;
    float random_float_value = minv + random_index * 0.1;
    return random_float_value;
}

float randy(){
    float minv = 1;
    float maxv = 23;
    int num = static_cast<int>((minv-maxv)*10) +1;
    int random_index = rand() % num;
    float random_float_value = minv + random_index * 0.1;
    return random_float_value;
}

int rantheta(){
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
    {1,0,1},
    {1,0,15},
    {0,1,1},
    {0,1,23},
    {0,1,12},
    {0,1,0.25},
    {0,1,23.75},
    {1,0,6.8},
    {1,0,9.2},
    {0,1,1.75},
    {0,1,22.25},
    {1,0,6.05},
    {1,0,9.95},
    {0,1,20.75},
    {0,1,3.25},
    {1,0,4.55},
    {1,0,11.45},
};

const vector<limits> Lim = {
    {1,1,1,23},
    {15,15,1,23},
    {1,15,1,1},
    {1,15,23,23},
    {1,15,12,12},
    {6.8,9.2,0.25,0.25},
    {6.8,9.2,23.75,23.75},  
    {6.8,6.8,0.25,1},//*
    {9.2,9.2,0.25,1},//*
    {6.05,9.95,1.75,1.75},
    {6.05,9.95,22.25,22.25},
    {6.05,6.05,1,1.75},//*
    {9.95,9.95,1,1.75},//*
    {4.55,11.45,20.75,20.75},
    {4.55,11.45,3.25,3.25},
    {4.55,4.55,1,3.25},//*
    {11.45,11.45,1,3.25},//*
    };

extern const limits Circle = {6,10,10,14};
extern const limits Quad1 = {1,1.75,1,1.75};
extern const limits Quad2 = {14.25,15,1,1.75};
extern const limits Quad3 = {1,1.75,22.25,23};
extern const limits Quad4 = {14.25,15,22.25,23};

void solve_for_centreCircle(WPoint p,float thetha, vector<linepoint> temp){
    float x=p.x,y=p.y;
    float a = -16 , b = -24, c=24;
    float m = tan(thetha);
    float c1 = y - m*x;
    float A = 1+m*m;
    float B = 2*m*c1+a+b*m;
    float C = c1*c1+c+b*c1;

    if(B>0){
        linepoint p1,p2;
        p1.x=(-B+sqrt(B*B-4*A*C))/2*A;
        p1.y = m*x+c1;
        p1.d = distance(x,y,p1.x,p1.y);
        p2.x=(-B-sqrt(B*B-4*A*C))/2*A;
        p2.y = m*x+c1;
        p2.d = distance(x,y,p2.x,p2.y);
        if(CompareByD(p2,p1))p1=p2;
        if (Circle.xmin <= p1.x && p1.x <= Circle.xmax && Circle.ymin <= p1.y && p1.y <= Circle.ymax)
            temp.push_back(p1);

    }
}

void solve_for_quad1(WPoint p, float thetha, vector<linepoint> temp){
    float x=p.x,y=p.y;
    float a = -2 , b = -2, c=1.4375;
    float m = tan(thetha);
    float c1 = y - m*x;
    float A = 1+m*m;
    float B = 2*m*c1+a+b*m;
    float C = c1*c1+c+b*c1;

    if(B>0){
        linepoint p1,p2;
        p1.x=(-B+sqrt(B*B-4*A*C))/2*A;
        p1.y = m*x+c1;
        p1.d = distance(x,y,p1.x,p1.y);
        p2.x=(-B-sqrt(B*B-4*A*C))/2*A;
        p2.y = m*x+c1;
        p2.d = distance(x,y,p2.x,p2.y);
        if(CompareByD(p2,p1))p1=p2;
        if (Quad1.xmin <= p1.x && p1.x <= Quad1.xmax && Quad1.ymin <= p1.y && p1.y <= Quad1.ymax)
            temp.push_back(p1);

    }
}

void solve_for_quad2(WPoint p,float thetha, vector<linepoint> temp){
    float x=p.x,y=p.y;
    float a = -30 , b = -2, c=225.4375;
    float m = tan(thetha);
    float c1 = y - m*x;
    float A = 1+m*m;
    float B = 2*m*c1+a+b*m;
    float C = c1*c1+c+b*c1;

    if(B>0){
        linepoint p1,p2;
        p1.x=(-B+sqrt(B*B-4*A*C))/2*A;
        p1.y = m*x+c1;
        p1.d = distance(x,y,p1.x,p1.y);
        p2.x=(-B-sqrt(B*B-4*A*C))/2*A;
        p2.y = m*x+c1;
        p2.d = distance(x,y,p2.x,p2.y);
        if(CompareByD(p2,p1))p1=p2;
        if (Quad2.xmin <= p1.x && p1.x <= Quad2.xmax && Quad2.ymin <= p1.y && p1.y <= Quad2.ymax)
            temp.push_back(p1);

    }
}

void solve_for_quad3(WPoint p,float thetha, vector<linepoint> temp){
    float x=p.x,y=p.y;
    float a = -2 , b = -46, c=529.4375;
    float m = tan(thetha);
    float c1 = y - m*x;
    float A = 1+m*m;
    float B = 2*m*c1+a+b*m;
    float C = c1*c1+c+b*c1;

    if(B>0){
        linepoint p1,p2;
        p1.x=(-B+sqrt(B*B-4*A*C))/2*A;
        p1.y = m*x+c1;
        p1.d = distance(x,y,p1.x,p1.y);
        p2.x=(-B-sqrt(B*B-4*A*C))/2*A;
        p2.y = m*x+c1;
        p2.d = distance(x,y,p2.x,p2.y);
        if(CompareByD(p2,p1))p1=p2;
        if (Quad3.xmin <= p1.x && p1.x <= Quad3.xmax && Quad3.ymin <= p1.y && p1.y <= Quad3.ymax)
            temp.push_back(p1);

    }
}

void solve_for_quad4(WPoint p,float thetha, vector<linepoint> temp){
    float x=p.x,y=p.y;
    float a = -30 , b = -46, c=753.4375;
    float m = tan(thetha);
    float c1 = y - m*x;
    float A = 1+m*m;
    float B = 2*m*c1+a+b*m;
    float C = c1*c1+c+b*c1;

    if(B>0){
        linepoint p1,p2;
        p1.x=(-B+sqrt(B*B-4*A*C))/2*A;
        p1.y = m*x+c1;
        p1.d = distance(x,y,p1.x,p1.y);
        p2.x=(-B-sqrt(B*B-4*A*C))/2*A;
        p2.y = m*x+c1;
        p2.d = distance(x,y,p2.x,p2.y);
        if(CompareByD(p2,p1))p1=p2;
        if (Quad4.xmin <= p1.x && p1.x <= Quad4.xmax && Quad4.ymin <= p1.y && p1.y <= Quad4.ymax)
            temp.push_back(p1);

    }
}


vector<linepoint> linepoints(WPoint a){
    float x=a.x,y=a.y;
    vector<linepoint> dists;
    for(float thetha =0; thetha<360; thetha+=0.5){
        float m = tan(thetha);
        float c1 = y - m*x;
        vector<linepoint> temp;
        for(size_t i=0;i<lines.size();i++){
            float a = lines[i].a;
            float b = lines[i].b;
            float c = lines[i].c;
            if(a+m*b!=0){
                linepoint p;
                p.x = (c-b*c1)/(a+m*b);
                p.y = (m*c+a*c1)/(a+m*b);
                p.d = distance(x,y,p.x,p.y);
                if(Lim[i].xmin<=p.x && p.x<=Lim[i].xmax && Lim[i].ymin<=p.y && p.y<=Lim[i].ymax)
                    temp.push_back(p);
                if (i == 7 && Lim[i].xmin <= p.x && p.x <= Lim[i].xmax && 23 <= p.y && p.y <= 23.75)
                    temp.push_back(p);
                if (i == 8 && Lim[i].xmin <= p.x && p.x <= Lim[i].xmax && 23 <= p.y && p.y <= 23.75)
                    temp.push_back(p);
                if (i == 11 && Lim[i].xmin <= p.x && p.x <= Lim[i].xmax && 22.25 <= p.y && p.y <= 23)
                    temp.push_back(p);
                if (i == 12 && Lim[i].xmin <= p.x && p.x <= Lim[i].xmax && 22.25 <= p.y && p.y <= 23)
                    temp.push_back(p);
                if (i == 15 && Lim[i].xmin <= p.x && p.x <= Lim[i].xmax && 20.75 <= p.y && p.y <= 23)
                    temp.push_back(p);
                if (i == 16 && Lim[i].xmin <= p.x && p.x <= Lim[i].xmax && 20.75 <= p.y && p.y <= 23)
                    temp.push_back(p);

            }
            else continue;
        }
        solve_for_centreCircle(a,thetha,temp);
        solve_for_quad1(a,thetha,temp); //bottomleft
        solve_for_quad2(a,thetha,temp); //bottomright
        solve_for_quad3(a,thetha,temp); //topleft
        solve_for_quad4(a,thetha,temp); //topright
        sort(temp.begin(),temp.end(),CompareByD);
        dists.push_back(temp[0]);
    }
    return dists;
}

linepoint findNearestPoint(WPoint a) {
    vector<linepoint> all_points = linepoints(a);

    sort(all_points.begin(), all_points.end(), CompareByD);
    linepoint x=all_points[0];
    all_points.clear();
    return x;
}

//uncomment to test the code
// int main() {
//     srand(static_cast<unsigned>(time(nullptr)));

//     WPoint a;
//     a.x = randx();
//     a.y = randy();

//     linepoint nearestPoint = findNearestPoint(a);

//     cout << "Given Point: {" << a.x << ", " << a.y << "}" << endl;
//     cout << "Nearest Point: {" << nearestPoint.x << ", " << nearestPoint.y << ", " << nearestPoint.d << "}" << endl;

//     return 0;
// }
