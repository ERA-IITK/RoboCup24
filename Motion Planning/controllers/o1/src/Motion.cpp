#include "Motion.hpp"

PID::PID() {
    min_out = 0, max_out = 0, min_integral = 0, max_integral = 0;
    integral = 0, derivative = 0;
    prev_error = 0;
}

double PID::calculatePID(double error, double min_max) {
    min_out = min_integral = -min_max;
    max_out = max_integral = min_max;

    double proportional = kp * error;
    integral += ki * error * time_pid;
    derivative = kd * (error - prev_error) / time_pid;

    if (integral > max_integral)
        integral = max_integral;
    else if (integral < min_integral)
        integral = min_integral;

    output_speed = proportional + integral + derivative;
    if (output_speed > max_out)
        output_speed = max_out;
    else if (output_speed < min_out)
        output_speed = min_out;

    prev_error = error;
    return output_speed;
}

void PID::setParam(double kp_, double ki_, double kd_) {
    kp = kp_;
    ki = ki_;
    kd = kd_;
}

void PID::reset() {
    integral = 0, derivative = 0;
    prev_error = 0;
    output_speed = 0;
}

Motion::Motion() {
    position_pid = new PID();
    yaw_pid = new PID();
}

void Motion::accelControl(Point2D *output_vel, Point2D &data) {
    // Static Velocity Buffer to store previous velocity.
    static double v_buffer[2];

    // Calculate Change in velocity
    double delta_v[2];
    delta_v[0] = data.x - v_buffer[0];
    delta_v[1] = data.y - v_buffer[1];

    // Convert the change to Polar Representation
    double r = sqrt(delta_v[0] * delta_v[0] + delta_v[1] * delta_v[1]);
    double theta = atan2(delta_v[1], delta_v[0]);
    // limit acceleration magnitude
    if (r > 3) r = 3;

    // Modify new velocity by converting back to Cartesian Representation
    v_buffer[0] += r * cos(theta);
    v_buffer[1] += r * sin(theta);

    // Set Output Parameters
    output_vel->x = v_buffer[0];
    output_vel->y = v_buffer[1];
    output_vel->theta = data.theta;

    // IC(output_vel->x, output_vel->y, output_vel->theta);
}

void Motion::basicMotion(double vx, double vy, double vtheta, double thetaRobot,
                         Point2D &output) {
    Point2D dataInput;
    double velout[3];
    velout[0] = cos(thetaRobot) * vx + sin(thetaRobot) * vy;
    velout[1] = -sin(thetaRobot) * vx + cos(thetaRobot) * vy;
    // IC(thetaRobot);
    dataInput.x = velout[0];
    dataInput.y = velout[1];
    dataInput.theta = vtheta;
    accelControl(&output, dataInput);
}

void Motion::positionAngularControl(double &errorX, double &errorY,
                                    double &errorTheta, double yaw,
                                    Point2D &outMotor, double nearestX,
                                    double nearestY, double minDistance,
                                    double currentX, double currentY,
                                    Point2D targetPos, int count,
                                    std::vector<Point2D> &path) {
    double angle = M_PI, speed = 80;
    int considered_point = 10;
    if (count + considered_point - 1 >= 0 &&
        count + considered_point + 1 <= path.size() - 1) {
        angle = acos(((path[count + considered_point + 1].x -
                       path[count + considered_point].x) *
                          (path[count + considered_point - 1].x -
                           path[count + considered_point].x) +
                      (path[count + considered_point + 1].y -
                       path[count + considered_point].y) *
                          (path[count + considered_point - 1].y -
                           path[count + considered_point].y)) /
                     (std::sqrt((path[count + considered_point + 1].x -
                                 path[count + considered_point].x) *
                                    (path[count + considered_point + 1].x -
                                     path[count + considered_point].x) +
                                (path[count + considered_point + 1].y -
                                 path[count + considered_point].y) *
                                    (path[count + considered_point + 1].y -
                                     path[count + considered_point].y)) *
                      std::sqrt((path[count + considered_point - 1].x -
                                 path[count + considered_point].x) *
                                    (path[count + considered_point - 1].x -
                                     path[count + considered_point].x) +
                                (path[count + considered_point - 1].y -
                                 path[count + considered_point].y) *
                                    (path[count + considered_point - 1].y -
                                     path[count + considered_point].y))));

        double sharpest_angle = 3, min_vel = 30, max_vel = 100;
        speed = min_vel + (max_vel - min_vel) * ((angle - sharpest_angle) /
                                                 (M_PI - sharpest_angle));
    }
    std::cout << "angle = " << angle << std::endl;

    if (count >= path.size() - 30) speed = 10;

    std::cout << "speed = " << speed << std::endl;
    std::cout << "minDistance = " << minDistance << std::endl;

    if (speed<50) position_pid->setParam(90, 0, 0);  // Set position_pid
    else if (speed<90) position_pid->setParam(80, 0, 0);
    else position_pid->setParam(75, 0, 0);

    yaw_pid->setParam(1, 0, 0);           // Set yaw_pid

    double proprotionalFactor = 80,
           normalSpeed = (speed < position_pid->calculatePID(minDistance, speed))
                             ? speed
                             : position_pid->calculatePID(minDistance, speed),
           tangentialSpeed =
               std::sqrt((0 > (speed * speed - normalSpeed * normalSpeed))
                             ? 0
                             : (speed * speed - normalSpeed * normalSpeed));

    std::cout << "normalSpeed = " << normalSpeed << std::endl;
    std::cout << "tangentialSpeed = " << tangentialSpeed << std::endl;

    error[0] =
        nearestX - currentX;  // Displacement along global X axis remaining
    error[1] = nearestY - currentY;  // Displacement global Y axis remaining
    error[2] = sqrt(error[0] * error[0] +
                    error[1] * error[1]);  // Net Displacement Magnitude
    error[3] = errorTheta;                 // Yaw Error

    // Apply PID on Error Magnitude
    output[2] = position_pid->calculatePID(error[2], 100);
    // Apply PID on Yaw Error
    output[3] = yaw_pid->calculatePID(error[3] * M_PI / 180.0, 5);

    double dotProduct =
        (cos(M_PI / 2 + atan2(error[1], error[0])) *
             cos(atan2(path[(count + 1 < path.size() - 1) ? count + 1
                                                          : path.size() - 1]
                               .y -
                           currentY,
                       path[(count + 1 < path.size() - 1) ? count + 1
                                                          : path.size() - 1]
                               .x -
                           currentX)) +
         sin(M_PI / 2 + atan2(error[1], error[0])) *
             sin(atan2(path[(count + 1 < path.size() - 1) ? count + 1
                                                          : path.size() - 1]
                               .y -
                           currentY,
                       path[(count + 1 < path.size() - 1) ? count + 1
                                                          : path.size() - 1]
                               .x -
                           currentX)));
    int dir = (dotProduct >= 0) ? 1 : -1;

    // Break into components
    output[0] =
        normalSpeed * cos(atan2(error[1], error[0])) +
        tangentialSpeed * cos(dir * M_PI / 2 + atan2(error[1], error[0]));
    output[1] =
        normalSpeed * sin(atan2(error[1], error[0])) +
        tangentialSpeed * sin(dir * M_PI / 2 + atan2(error[1], error[0]));

    // if (std::abs(dotProduct) < 0.3) {
    //     output[0] = normalSpeed * cos(atan2(error[1], error[0]));
    //     output[1] = normalSpeed * sin(atan2(error[1], error[0]));
    // }

    // IC(output[0], output[1], output[2], output[3]);
    basicMotion(output[0], output[1], output[3], yaw, outMotor);
}
