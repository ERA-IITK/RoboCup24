#include <cmath>

#include "Motion.hpp"
#include "PathPlanner.hpp"
#include "Robot.hpp"
#include "Visualize.hpp"
#include "iostream"
#include "webots/GPS.hpp"
#include "webots/InertialUnit.hpp"
#include "webots/Motor.hpp"
#include "webots/PositionSensor.hpp"
#include "webots/Robot.hpp"
#include "webots/Supervisor.hpp"

using namespace webots;
const int maxNumberCord = 10000;
const int refresh_factor = 2.0;

Motor *motor[4];
PositionSensor *enc[4];
InertialUnit *imu_device;
std::vector<double> prevPulse{0, 0, 0, 0};
std::vector<double> encData{0, 0, 0, 0};

using namespace robot;
Motion *mot = new Motion();
Point2D nowPos(0, 0, 0);
int count1 = 0;

Supervisor *robotSup = new Supervisor();
Node *my_root = robotSup->getRoot();
Visualize *window = new Visualize(1600);

// Function to check if a point lies within the extended region of a line
bool isok(const Point2D &p1, const Point2D &p2, const Point2D &testPoint) {
    // finds perpendicular isstance between line joining p1 and p2 and the
    // obstacles
    double distance =
        std::abs((p2.y - p1.y) * testPoint.x - (p2.x - p1.x) * testPoint.y +
                 p2.x * p1.y - p2.y * p1.x) /
        std::sqrt(std::pow(p2.y - p1.y, 2) + std::pow(p2.x - p1.x, 2));

    return distance <
           0.45;  // Check if the distance is within the extended region
}

// function to find the point closest to current position
int findclosestpoint(std::vector<Point2D> &targetPos, Point2D &nowPos) {
    double min = 1000000;
    int idx = 0;
    for (int i = 0; i < targetPos.size() - 1;
         i++)  // excluding last point and start
    {
        if (sqrt((targetPos[i].x - nowPos.x) * (targetPos[i].x - nowPos.x) +
                 (targetPos[i].y - nowPos.y) * (targetPos[i].y - nowPos.y)) <
            min) {
            min =
                sqrt((targetPos[i].x - nowPos.x) * (targetPos[i].x - nowPos.x) +
                     (targetPos[i].y - nowPos.y) * (targetPos[i].y - nowPos.y));
            idx = i;
        }
    }
    return idx;
}

void get_Trajectory(std::vector<Point2D> &path, Point2D &outputPID,
                    Point2D &nowPos, wheelAngularVel &outInvers, double yaw,
                    std::vector<Point2D> &obstacles, Point2D &ball) {
    window->visualizeGame(path, nowPos, count1, yaw, obstacles, ball);

    path[count1].theta = (180.0 / 3.14159) * atan2((ball.y - path[count1].y),
                                                   (ball.x - path[count1].x));

    double errorX = path[count1].x - nowPos.x;
    double errorY = path[count1].y - nowPos.y;

    double dist = sqrt(errorX * errorX + errorY * errorY);
    double errTheta = path[count1].theta - (nowPos.theta * 180 / M_PI);

    // Limit errTheta to -180 to +180 degrees
    if (errTheta > 180) errTheta -= 360;
    if (errTheta < -180) errTheta += 360;

    double nearestX, nearestY,
        minDistance = std::numeric_limits<double>::infinity();
    for (int j = 0; j < path.size(); j++) {
        auto i = path[j];
        double dist = std::sqrt((i.x - nowPos.x) * (i.x - nowPos.x) +
                                (i.y - nowPos.y) * (i.y - nowPos.y));
        if (dist < minDistance) {
            minDistance = dist;
            nearestX = i.x;
            nearestY = i.y;
            count1 = j;
        }
    }

    // IF using IMU as orientation
    mot->positionAngularControl(errorX, errorY, errTheta, yaw, outputPID,
                                nearestX, nearestY, minDistance, nowPos.x,
                                nowPos.y, path.back(), count1, path);

    // If using Odometry orientation
    // mot->PositionAngularControl(errorX, errorY, errTheta, nowPos.theta,
    //                             outputPID);

    RobotKinematic::getInstance()->inverseKinematics(
        outInvers, outputPID.x, outputPID.y, outputPID.theta);

    // if (dist < 0.15 && fabs(errTheta) < 3) {
    //     mot->position_pid->reset();
    //     mot->yaw_pid->reset();
    //     count1++;
    //     IC(count1);
    // }
    if (count1 > path.size() - 1) {
        count1 = path.size() - 1;
    }
}

void setVel(wheelAngularVel &outMotor) {
    motor[0]->setVelocity(outMotor.w1);
    motor[1]->setVelocity(outMotor.w2);
    motor[2]->setVelocity(outMotor.w3);
    motor[3]->setVelocity(outMotor.w4);
}

static void create_line() {
    Node *existing_line = robotSup->getFromDef("TRACK");
    // Node *myRobot = super->getFromDef("OMNI_WHEELS");
    // Field *translation_field = myRobot->getField("translation");

    if (existing_line) existing_line->remove();

    int i;
    std::string track_string =
        "";  // Initialize a big string which will contain the TRAIL node.
    // Create the TRAIL Shape.
    track_string += "DEF TRACK Shape {\n";
    track_string += "  appearance Appearance {\n";
    track_string += "    material Material {\n";
    track_string += "      diffuseColor 0 0 0\n";
    track_string += "      emissiveColor 0 0 0\n";
    track_string += "    }\n";
    track_string += "  }\n";
    track_string += "  geometry DEF TRACK_LINE_SET IndexedLineSet {\n";
    track_string += "    coord Coordinate {\n";
    track_string += "      point [\n";
    for (i = 0; i < maxNumberCord; ++i) track_string += "      0 0 0\n";
    track_string += "      ]\n";
    track_string += "    }\n";
    track_string += "    coordIndex [\n";
    for (i = 0; i < maxNumberCord; ++i) track_string += "      0 0 -1\n";
    track_string += "    ]\n";
    track_string += "  }\n";
    track_string += "}\n";

    // Import TRAIL and append it as the world root nodes.

    Field *field_children = my_root->getField("children");

    field_children->importMFNodeFromString(-1, track_string);
}

void getOtherPositionYaw(std::string name, Point2D &output) {
    Node *B1 = robotSup->getFromDef(name);
    // if (!*B1) {
    //     return;
    // }
    const double *position = B1->getPosition();
    double x = position[0];
    double y = position[1];

    const double *q = B1->getOrientation();  // We get the quaternion in {w,
                                             // x, y, z} format

    // Normalise (in case it already hasn't been)
    double q_mod =
        std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    double q_norm[4] = {q[0] / q_mod, q[1] / q_mod, q[2] / q_mod, q[3] / q_mod};
    // Calculate yaw (about x-axis) from -pi to +pi. The reference x axis is
    // towards the right.
    double siny_cosp = 2 * (q_norm[0] * q_norm[3] + q_norm[1] * q_norm[2]);
    double cosy_cosp = 1 - 2 * (q_norm[2] * q_norm[2] + q_norm[3] * q_norm[3]);
    double yaw = std::atan2(siny_cosp, cosy_cosp);

    if (q[0] <= 0) {
        if (q[3] >= 0) {
            // Second Quadrant
            yaw += M_PI;
        } else {
            // Third Quadrant
            yaw -= M_PI;
        }
    }

    // yaw = yaw * (180.0 / M_PI);  // Convert radians to degrees

    output.x = x;
    output.y = y;
    output.theta = yaw;
}

int main(int argc, char **argv) {
    int flag = 0;  // flag used in while loop for determining if path should be
                   // replanned
    Point2D Ball;
    long long int count1 = 0;
    long long int count2 = 1;

    std::vector<Point2D> obstacles(5);
    std::vector<Point2D> targetPos;
    std::vector<PointPair> points;  // from output.txt

    // OMPL Setup Parameter
    double runTime = 0.01;
    optimalPlanner plannerType = PLANNER_RRTSTAR;
    planningObjective objectiveType = OBJECTIVE_PATHLENGTH;
    std::string outputFile = "output.txt";

    // Begin Controller
    wheelAngularVel outInvers;
    double dt = 0.0;
    Point2D outputPID(0, 0, 0.654);
    int stepTime = robotSup->getBasicTimeStep();
    stepTime *= refresh_factor;

    imu_device = robotSup->getInertialUnit("IMU");
    imu_device->enable(stepTime);

    GPS *gps_dev = robotSup->getGPS("GPS");
    gps_dev->enable(stepTime);

    // Motor Initial
    char motorNames[4][8] = {"wheel1", "wheel2", "wheel3", "wheel4"};

    Node *target_line = robotSup->getFromDef("OMNI_WHEELS_4");

    // SET the first Position our Robot
    RobotKinematic::getInstance()->setInitialPosition(
        0, 0, 0);  // Set Position x = 0, y = 0, theta = 0

    // Create Line in webots environment to see actual path traced
    create_line();

    Node *trail_line_set = robotSup->getFromDef("TRACK_LINE_SET");
    Field *coord_field = trail_line_set->getField("coord");

    Node *coordinate_node = coord_field->getSFNode();
    Field *pointField = coordinate_node->getField("point");
    Field *coord_index_field = trail_line_set->getField("coordIndex");

    int index = 0;
    bool first_step = true;
    // Track line end-----------

    // set Motor to webots
    for (int i = 0; i < 4; i++) {
        motor[i] = robotSup->getMotor(motorNames[i]);
        motor[i]->setPosition(INFINITY);
        motor[i]->setVelocity(0);
    }

    // Encoder Initial
    char encNames[4][8] = {"pw1", "pw2", "pw3", "pw4"};

    for (int i = 0; i < 4; i++) {
        enc[i] = robotSup->getPositionSensor(encNames[i]);
        enc[i]->enable(stepTime);
    }

    // std::vector<Point2D> circVec;
    // for (int i = 0; i < 360; i += 10) {
    //     double x_ = 1 * cos((double)i * M_PI / 180);
    //     double y_ = 1 * sin((double)i * M_PI / 180);
    //     circVec.push_back(Point2D(x_, y_, 0));
    // }
    // std::vector<Point2D> targetPos;
    // for (auto &ptr : points) {
    //     targetPos.push_back(Point2D(ptr.first, ptr.second, 0));
    // }

    // std::vector<Point2D> targetPos = {Point2D(-10, 0, 0), Point2D(2, -1,0),
    //                                   Point2D(0, -2, 0), Point2D(0, 2, 0)};
    Point2D gps_pos{0, 0, 0};

    while (robotSup->step(stepTime) != -1) {
        const double *robotTranslations = target_line->getPosition();
        const double *orientation = imu_device->getRollPitchYaw();
        const double yaw = orientation[2];

        // add New Position
        pointField->setMFVec3f(index, robotTranslations);

        // update line track
        if (index > 0) {
            coord_index_field->setMFInt32(3 * (index - 1), index - 1);
            coord_index_field->setMFInt32(3 * (index - 1) + 1, index);
        } else if (index == 0 && first_step == false) {
            coord_index_field->setMFInt32(3 * (maxNumberCord - 1), 0);
            coord_index_field->setMFInt32(3 * (maxNumberCord - 1) + 1,
                                          (maxNumberCord - 1));
        }

        // GPS for testing
        const double *gps_raw = gps_dev->getValues();
        gps_pos.x = gps_raw[0];
        gps_pos.y = gps_raw[1];
        gps_pos.theta = yaw;

        nowPos = robot::RobotKinematic::getInstance()->getPos();
        IC(nowPos.x, nowPos.y, nowPos.theta);

        getOtherPositionYaw(
            "Ball",
            Ball);  // The proto does not have the DEF in it. You
                    // need to set it from the Webots scene tree.
        for (int i = 0; i < 5; i++) {
            getOtherPositionYaw("B" + std::to_string(i + 1), obstacles[i]);
        }

        if (count2 % 5 == 0) {
            int idx = findclosestpoint(targetPos,
                                       nowPos);  // excluding the last point
            for (auto &it : obstacles) {
                if (idx + 1 < obstacles.size() && idx - 1 > -1) {
                    if (!isok(targetPos[idx], targetPos[idx + 1], it) &&
                        !isok(targetPos[idx - 1], targetPos[idx], it))
                        count1++;
                } else if (idx + 1 >= obstacles.size()) {
                    if (!isok(targetPos[idx - 1], targetPos[idx], it)) count1++;
                } else if (idx - 1 < -1) {
                    if (!isok(targetPos[idx], targetPos[idx + 1], it)) count1++;
                }
            }
            if (count1 != obstacles.size()) flag = 1;
            std::cout << count1 << std::endl;
            count1 = 0;
        }

        if (flag || count2 == 1) {
            plan(runTime, 22, 14, obstacles, plannerType, objectiveType,
                 outputFile, nowPos);  // Hardcoded field parameters.
            points.resize(0);
            targetPos.resize(0);
            points = readPointsFromFile();

            for (auto &ptr : points) {
                int angle = (180.0 / 3.14159) *
                            atan2((Ball.y - ptr.second), (Ball.x - ptr.first));
                targetPos.push_back(Point2D(ptr.first, ptr.second, angle));
            }
            flag = 0;
        }

        count2++;
        count2 = count2 % 1000000000000000000;

        get_Trajectory(targetPos, outputPID, nowPos, outInvers, yaw, obstacles,
                       Ball);

        setVel(outInvers);

        // Assign Enc val
        for (int i = 0; i < 4; i++) {
            RobotKinematic::getInstance()->encData[i] = enc[i]->getValue();
        }
        RobotKinematic::getInstance()->calculateOdometry(yaw);

        // unset next indices
        coord_index_field->setMFInt32(3 * index, index);
        coord_index_field->setMFInt32(3 * index + 1, index);

        if (robotSup->step(robotSup->getBasicTimeStep()) == -1) break;
        first_step = false;
        index++;
        index = index % maxNumberCord;
    }

    robotSup->simulationQuit(EXIT_SUCCESS);
    delete robotSup;

    return 0;
}