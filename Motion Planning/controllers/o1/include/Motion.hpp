#ifndef MOTION_H
#define MOTION_H

#include "Coordinate.hpp"
#include "icecream.hpp"
#include "vector"

// PID Controller
class PID {
   private:
    double min_out, max_out, min_integral, max_integral;
    double output_speed;
    double proportional, integral, derivative, prev_error;
    double kp, ki, kd;
    const double time_pid = 0.01;

   public:
    PID();
    double calculatePID(double error, double min_max);
    void setParam(double kp_, double ki_, double kd_);
    void reset();
};

// Control the Position and Yaw of the robot using PID Controllers.
class Motion {
   private:
    std::vector<double> output = {0, 0, 0, 0};
    std::vector<double> error = {0, 0, 0, 0};
    void basicMotion(double vx, double vy, double vtheta, double thetaRobot,
                     Point2D &output);
    void accelControl(Point2D *out, Point2D &data);

   public:
    PID *position_pid, *yaw_pid;
    Motion();
    void positionAngularControl(double &errorX, double &errorY,
                                double &errorTheta, double yaw,
                                Point2D &outMotor, double nearestX,
                                double nearestY, double minDistance,
                                double currentX, double currentY,
                                Point2D targetPos, int count,
                                std::vector<Point2D> &path);
};

#endif
