#ifndef ROBOT_H
#define ROBOT_H

#include "Coordinate.hpp"
#include "icecream.hpp"
#include "math.h"
#include "vector"

namespace robot {

class RobotKinematic {
   private:
    Point2D pos = {0, 0, 0};
    Point2D vel = {0, 0, 0};

    static RobotKinematic *instance;
    double a = 45;  // Angle between wheels
    double L =
        41.6;  // radius from center to wheel (cm); BaseLength 4,795.94 MM;
    double r_wheel = 6;  // wheel radius (cm)
    double circumference = 2 * M_PI * r_wheel;
    std::vector<double> Venc = {0, 0, 0, 0};
    std::vector<double> prevEnc = {0, 0, 0, 0};

   public:
    static RobotKinematic *getInstance();
    std::vector<double> encData = {0, 0, 0, 0};

    RobotKinematic(){};
    Point2D getPos();
    void forwardKinematics(Point2D &outForward, double s1, double s2, double s3,
                           double s4);
    void calculateOdometry(const double yaw);
    void inverseKinematics(wheelAngularVel &outputInverse, double vx, double vy,
                           double theta);
    double angleNormalize(double angle);

    void setInitialPosition(double x, double y, double theta);
};
};  // namespace robot

#endif