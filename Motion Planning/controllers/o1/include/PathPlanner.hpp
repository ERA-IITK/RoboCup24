#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
// For ompl::msg::setLogLevel
#include "ompl/util/Console.h"

// The supported optimal planners, in alphabetical order
#include <ompl/geometric/planners/cforest/CForest.h>
#include <ompl/geometric/planners/fmt/BFMT.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/informedtrees/AITstar.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/SORRTstar.h>

// For boost program options
#include <boost/program_options.hpp>
// For string comparison (boost::iequals)
#include <boost/algorithm/string.hpp>
// For std::make_shared
#include <fstream>
#include <iostream>
#include <memory>
#include <vector>

#include "spline.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

using namespace std;

bool sortbysec(const pair<int, int> &a, const pair<int, int> &b) {
    return (a.second > b.second);
}

// OMPL Code uptil main
// An enum of supported optimal planners, alphabetical order
enum optimalPlanner {
    PLANNER_AITSTAR,
    PLANNER_BFMTSTAR,
    PLANNER_BITSTAR,
    PLANNER_CFOREST,
    PLANNER_FMTSTAR,
    PLANNER_INF_RRTSTAR,
    PLANNER_PRMSTAR,
    PLANNER_RRTSTAR,
    PLANNER_SORRTSTAR,
};

// An enum of the supported optimization objectives, alphabetical order
enum planningObjective {
    OBJECTIVE_PATHCLEARANCE,
    OBJECTIVE_PATHLENGTH,
    OBJECTIVE_THRESHOLDPATHLENGTH,
    OBJECTIVE_WEIGHTEDCOMBO
};

struct PointWithRadius {
    double x;  // x-coordinate
    double y;  // y-coordinate
    double R;  // radius
};

std::vector<Point2D> obstacles;

// Define a pair for the points
using PointPair = std::pair<double, double>;

std::vector<PointPair> readPointsFromFile() {
    std::string filename = "output1.txt";
    std::vector<PointPair> points;

    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Unable to open file " << filename << std::endl;
        return points;  // Return empty vector on error
    }

    double x, y;
    while (file >> x >> y) {
        points.push_back({x, y});
    }

    file.close();

    if (file.fail() && !file.eof()) {
        std::cerr << "Error reading data from file " << filename << std::endl;
        points.clear();  // Clear vector on error
    }

    // for (const auto &p : points) {
    //     std::cout << p.first << " " << p.second << std::endl;
    // }

    return points;
}

// Parse the command-line arguments
bool argParse(int argc, char **argv, double *runTimePtr,
              optimalPlanner *plannerPtr, planningObjective *objectivePtr,
              std::string *outputFilePtr);

// Our "collision checker". For this demo, our robot's state space
// lies in [0,1]x[0,1], with a circular obstacle of radius 0.25
// centered at (0.5,0.5). Any states lying in this circular region are
// considered "in collision".
class ValidityChecker : public ob::StateValidityChecker {
   public:
    ValidityChecker(const ob::SpaceInformationPtr &si)
        : ob::StateValidityChecker(si) {}

    // Returns whether the given state's position overlaps the
    // circular obstacle
    bool isValid(const ob::State *state) const override {
        return this->clearance(state) > 0.0;
    }

    // Returns the distance from the given state's position to the
    // boundary of the circular obstacle.
    double clearance(const ob::State *state) const override {
        // We know we're working with a RealVectorStateSpace in this
        // example, so we downcast state into the specific type.
        const auto *state2D = state->as<ob::RealVectorStateSpace::StateType>();

        // Extract the robot's (x,y) position from its state
        double x = state2D->values[0];
        double y = state2D->values[1];

        // Distance formula between two points, offset by the circle's
        // radius r

        double r = 0.9;
        // Dynamically allocate an array of PointWithRadius
        std::vector<PointWithRadius> v;

        for (int i = 0; i < obstacles.size(); i++) {
            v.push_back({obstacles[i].x, obstacles[i].y, r});
        }

        int count = 0;
        for (auto &ptr : v) {
            if (sqrt((x - ptr.x) * (x - ptr.x) + (y - ptr.y) * (y - ptr.y)) -
                    ptr.R >
                0.0)
                count++;
        }
        if (count == v.size())
            return 1.0;
        else
            return -1.0;
    }
};

ob::OptimizationObjectivePtr getPathLengthObjective(
    const ob::SpaceInformationPtr &si);

ob::OptimizationObjectivePtr getThresholdPathLengthObj(
    const ob::SpaceInformationPtr &si);

ob::OptimizationObjectivePtr getClearanceObjective(
    const ob::SpaceInformationPtr &si);

ob::OptimizationObjectivePtr getBalancedObjective1(
    const ob::SpaceInformationPtr &si);

ob::OptimizationObjectivePtr getBalancedObjective2(
    const ob::SpaceInformationPtr &si);

ob::OptimizationObjectivePtr getPathLengthObjWithCostToGo(
    const ob::SpaceInformationPtr &si);

ob::PlannerPtr allocatePlanner(ob::SpaceInformationPtr si,
                               optimalPlanner plannerType) {
    switch (plannerType) {
        case PLANNER_AITSTAR: {
            return std::make_shared<og::AITstar>(si);
            break;
        }
        case PLANNER_BFMTSTAR: {
            return std::make_shared<og::BFMT>(si);
            break;
        }
        case PLANNER_BITSTAR: {
            return std::make_shared<og::BITstar>(si);
            break;
        }
        case PLANNER_CFOREST: {
            return std::make_shared<og::CForest>(si);
            break;
        }
        case PLANNER_FMTSTAR: {
            return std::make_shared<og::FMT>(si);
            break;
        }
        case PLANNER_INF_RRTSTAR: {
            return std::make_shared<og::InformedRRTstar>(si);
            break;
        }
        case PLANNER_PRMSTAR: {
            return std::make_shared<og::PRMstar>(si);
            break;
        }
        case PLANNER_RRTSTAR: {
            return std::make_shared<og::RRTstar>(si);
            break;
        }
        case PLANNER_SORRTSTAR: {
            return std::make_shared<og::SORRTstar>(si);
            break;
        }
        default: {
            OMPL_ERROR(
                "Planner-type enum is not implemented in allocation function.");
            return ob::PlannerPtr();  // Address compiler warning re: no return
                                      // value.
            break;
        }
    }
}

ob::OptimizationObjectivePtr allocateObjective(
    const ob::SpaceInformationPtr &si, planningObjective objectiveType) {
    switch (objectiveType) {
        case OBJECTIVE_PATHCLEARANCE:
            return getClearanceObjective(si);
            break;
        case OBJECTIVE_PATHLENGTH:
            return getPathLengthObjective(si);
            break;
        case OBJECTIVE_THRESHOLDPATHLENGTH:
            return getThresholdPathLengthObj(si);
            break;
        case OBJECTIVE_WEIGHTEDCOMBO:
            return getBalancedObjective1(si);
            break;
        default:
            OMPL_ERROR(
                "Optimization-objective enum is not implemented in allocation "
                "function.");
            return ob::OptimizationObjectivePtr();
            break;
    }
}

void plan(double runTime, double A, double B, std::vector<Point2D> &obs,
          optimalPlanner plannerType, planningObjective objectiveType,
          const std::string &outputFile, Point2D &nowPos) {
    // Construct the robot state space in which we're planning. We're
    // planning in the playable field, a subset of R^2 (a rectangle of
    // dimensions A x B)
    obstacles = obs;
    auto space(std::make_shared<ob::RealVectorStateSpace>(2));

    // Set the bounds of space to be in [0,1].
    // space->setBounds(0.0, 1.0);
    // Set the bounds of space to be in [-11, 11] for both dimensions.
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(0, -A / 2);
    bounds.setHigh(0, A / 2);
    bounds.setLow(1, -B / 2);
    bounds.setHigh(1, B / 2);
    space->setBounds(bounds);

    // Construct a space information instance for this state space
    auto si(std::make_shared<ob::SpaceInformation>(space));

    // Set the object used to check which states in the space are valid
    si->setStateValidityChecker(std::make_shared<ValidityChecker>(si));

    si->setup();

    // Set our robot's starting state to be the bottom-left corner of
    // the environment, or (0,0).
    ob::ScopedState<> start(space);
    start->as<ob::RealVectorStateSpace::StateType>()->values[0] = nowPos.x;
    start->as<ob::RealVectorStateSpace::StateType>()->values[1] = nowPos.y;

    // Set our robot's goal state to be the top-right corner of the
    // environment, or (1,1).
    ob::ScopedState<> goal(space);
    goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = 7.0;
    goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = 5.0;

    // Create a problem instance
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));

    // Set the start and goal states
    pdef->setStartAndGoalStates(start, goal);

    // Create the optimization objective specified by our command-line argument.
    // This helper function is simply a switch statement.
    pdef->setOptimizationObjective(allocateObjective(si, objectiveType));

    // Construct the optimal planner specified by our command line argument.
    // This helper function is simply a switch statement.
    ob::PlannerPtr optimizingPlanner = allocatePlanner(si, plannerType);

    // Set the problem instance for our planner to solve
    optimizingPlanner->setProblemDefinition(pdef);
    optimizingPlanner->setup();

    // attempt to solve the planning problem in the given runtime
    ob::PlannerStatus solved = optimizingPlanner->solve(runTime);

    if (solved) {
        // Output the length of the path found
        // std::cout
        // << optimizingPlanner->getName()
        // << " found a solution of length "
        // << pdef->getSolutionPath()->length()
        // << " with an optimization objective value of "
        // << pdef->getSolutionPath()->cost(pdef->getOptimizationObjective()) <<
        // std::endl;

        // If a filename was specified, output the path as a matrix to
        // that file for visualization
        // if (outputFile.empty()) {
        std::ofstream outFile(outputFile);
        std::static_pointer_cast<og::PathGeometric>(pdef->getSolutionPath())
            ->printAsMatrix(outFile);
        outFile.close();
        // }

        // const char* command = "python3 smooth.py";
        // // Execute the Python script from the C++ program
        // int result = std::system(command);

        // Read waypoints from the "output.txt" file
        std::ifstream file("output.txt");
        if (!file.is_open()) {
            std::cerr << "Error opening file." << std::endl;
        }

        std::vector<double> x_values, y_values;
        double x, y;
        while (file >> x >> y) {
            x_values.push_back(x);
            y_values.push_back(y);
        }
        file.close();

        std::vector<double> T(x_values.size());

        // std::vector< std::pair<double, std::pair<double, double> > > v;

        // double start_x=x_values.front(), end_x=x_values.back();

        // if (start_x > end_x)
        // {
        //     std::reverse(x_values.begin(),x_values.end());
        //     std::reverse(y_values.begin(),y_values.end());
        // }

        // tk::spline s(x_values,y_values,tk::spline::cspline);

        // Array to store robot positions
        std::vector<std::pair<double, double>> robot_positions;

        if (x_values.size() > 2) {
            T[0] = 0;
            for (int i = 1; i < x_values.size(); i++) {
                T[i] = T[i - 1] + sqrt((x_values[i] - x_values[i - 1]) *
                                           (x_values[i] - x_values[i - 1]) +
                                       (y_values[i] - y_values[i - 1]) *
                                           (y_values[i] - y_values[i - 1]));
            }
            // setup splines for x and y coordinate
            tk::spline sx(T, x_values), sy(T, y_values);

            double n = 200, diff = T.back() / n;
            for (int i = 0; i < n; i++) {
                double t = diff * i;
                robot_positions.push_back({sx(t), sy(t)});
            }
            robot_positions.push_back({sx(T.back()), sy(T.back())});
        } else {
            for (int i = 0; i < x_values.size(); i++) {
                robot_positions.push_back({x_values[i], y_values[i]});
            }
        }

        // Display the array of robot positions
        // std::cout << "Robot Positions:\n";
        // for (const auto &pos : robot_positions) {
        //     std::cout << "(" << pos.first << ", " << pos.second << ")\n";
        // }

        // Write the array to the text file
        std::ofstream output_file("output1.txt");
        if (!output_file.is_open()) {
            std::cerr << "Error opening output file." << std::endl;
        }

        for (const auto &pos : robot_positions) {
            output_file << pos.first << " " << pos.second << "\n";
        }
        output_file.close();

        std::cout << "The array has been written to output1.txt\n";

    } else
        std::cout << "No solution found." << std::endl;
}

ob::OptimizationObjectivePtr getPathLengthObjective(
    const ob::SpaceInformationPtr &si) {
    return std::make_shared<ob::PathLengthOptimizationObjective>(si);
}

ob::OptimizationObjectivePtr getThresholdPathLengthObj(
    const ob::SpaceInformationPtr &si) {
    auto obj(std::make_shared<ob::PathLengthOptimizationObjective>(si));
    obj->setCostThreshold(ob::Cost(1.51));
    return obj;
}

class ClearanceObjective : public ob::StateCostIntegralObjective {
   public:
    ClearanceObjective(const ob::SpaceInformationPtr &si)
        : ob::StateCostIntegralObjective(si, true) {}

    // Our requirement is to maximize path clearance from obstacles,
    // but we want to represent the objective as a path cost
    // minimization. Therefore, we set each state's cost to be the
    // reciprocal of its clearance, so that as state clearance
    // increases, the state cost decreases.
    ob::Cost stateCost(const ob::State *s) const override {
        return ob::Cost(1 / (si_->getStateValidityChecker()->clearance(s) +
                             std::numeric_limits<double>::min()));
    }
};

ob::OptimizationObjectivePtr getClearanceObjective(
    const ob::SpaceInformationPtr &si) {
    return std::make_shared<ClearanceObjective>(si);
}

ob::OptimizationObjectivePtr getBalancedObjective1(
    const ob::SpaceInformationPtr &si) {
    auto lengthObj(std::make_shared<ob::PathLengthOptimizationObjective>(si));
    auto clearObj(std::make_shared<ClearanceObjective>(si));
    auto opt(std::make_shared<ob::MultiOptimizationObjective>(si));
    opt->addObjective(lengthObj, 10.0);
    opt->addObjective(clearObj, 1.0);

    return ob::OptimizationObjectivePtr(opt);
}

ob::OptimizationObjectivePtr getBalancedObjective2(
    const ob::SpaceInformationPtr &si) {
    auto lengthObj(std::make_shared<ob::PathLengthOptimizationObjective>(si));
    auto clearObj(std::make_shared<ClearanceObjective>(si));

    return 10.0 * lengthObj + clearObj;
}

ob::OptimizationObjectivePtr getPathLengthObjWithCostToGo(
    const ob::SpaceInformationPtr &si) {
    auto obj(std::make_shared<ob::PathLengthOptimizationObjective>(si));
    obj->setCostToGoHeuristic(&ob::goalRegionCostToGo);
    return obj;
}

bool argParse(int argc, char **argv, double *runTimePtr,
              optimalPlanner *plannerPtr, planningObjective *objectivePtr,
              std::string *outputFilePtr) {
    namespace bpo = boost::program_options;

    // Declare the supported options.
    bpo::options_description desc("Allowed options");
    desc.add_options()("help,h", "produce help message")(
        "runtime,t", bpo::value<double>()->default_value(1.0),
        "(Optional) Specify the runtime in seconds. Defaults to 1 and must be "
        "greater than 0.")("planner,p",
                           bpo::value<std::string>()->default_value("RRTstar"),
                           "(Optional) Specify the optimal planner to use, "
                           "defaults to RRTstar if "
                           "not given. Valid options are AITstar, BFMTstar, "
                           "BITstar, CForest, "
                           "FMTstar, InformedRRTstar, PRMstar, RRTstar, and "
                           "SORRTstar.")  // Alphabetical
                                          // order
        ("objective,o", bpo::value<std::string>()->default_value("PathLength"),
         "(Optional) Specify the optimization objective, defaults to "
         "PathLength if not given. Valid options are PathClearance, "
         "PathLength, ThresholdPathLength, and "
         "WeightedLengthAndClearanceCombo.")  // Alphabetical order
        ("file,f", bpo::value<std::string>()->default_value("output.txt"),
         "(Optional) Specify an output path for the found solution path.");
    // ("info,i", bpo::value<unsigned int>()->default_value(0u), "(Optional) Set
    // the OMPL log level. 0 for WARN, 1 for INFO, 2 for DEBUG. Defaults to
    // WARN.");
    bpo::variables_map vm;
    bpo::store(bpo::parse_command_line(argc, argv, desc), vm);
    bpo::notify(vm);

    // Check if the help flag has been given:
    if (vm.count("help") != 0u) {
        std::cout << desc << std::endl;
        return false;
    }

    // Set the log-level
    // unsigned int logLevel = vm["info"].as<unsigned int>();

    // Switch to setting the log level:
    // if (logLevel == 0u)
    // {
    // ompl::msg::setLogLevel(ompl::msg::LOG_WARN);
    // }
    // else if (logLevel == 1u)
    // {
    // ompl::msg::setLogLevel(ompl::msg::LOG_INFO);
    // }
    // else if (logLevel == 2u)
    // {
    // ompl::msg::setLogLevel(ompl::msg::LOG_DEBUG);
    // }
    // else
    // {
    // std::cout << "Invalid log-level integer." << std::endl << std::endl <<
    // desc << std::endl; return false;
    // }

    // Get the runtime as a double
    *runTimePtr = vm["runtime"].as<double>();

    // Sanity check
    if (*runTimePtr <= 0.0) {
        std::cout << "Invalid runtime." << std::endl
                  << std::endl
                  << desc << std::endl;
        return false;
    }

    // Get the specified planner as a string
    std::string plannerStr = vm["planner"].as<std::string>();

    // Map the string to the enum
    if (boost::iequals("AITstar", plannerStr)) {
        *plannerPtr = PLANNER_AITSTAR;
    } else if (boost::iequals("BFMTstar", plannerStr)) {
        *plannerPtr = PLANNER_BFMTSTAR;
    } else if (boost::iequals("BITstar", plannerStr)) {
        *plannerPtr = PLANNER_BITSTAR;
    } else if (boost::iequals("CForest", plannerStr)) {
        *plannerPtr = PLANNER_CFOREST;
    } else if (boost::iequals("FMTstar", plannerStr)) {
        *plannerPtr = PLANNER_FMTSTAR;
    } else if (boost::iequals("InformedRRTstar", plannerStr)) {
        *plannerPtr = PLANNER_INF_RRTSTAR;
    } else if (boost::iequals("PRMstar", plannerStr)) {
        *plannerPtr = PLANNER_PRMSTAR;
    } else if (boost::iequals("RRTstar", plannerStr)) {
        *plannerPtr = PLANNER_RRTSTAR;
    } else if (boost::iequals("SORRTstar", plannerStr)) {
        *plannerPtr = PLANNER_SORRTSTAR;
    } else {
        std::cout << "Invalid planner string." << std::endl
                  << std::endl
                  << desc << std::endl;
        return false;
    }

    // Get the specified objective as a string
    std::string objectiveStr = vm["objective"].as<std::string>();

    // Map the string to the enum
    if (boost::iequals("PathClearance", objectiveStr)) {
        *objectivePtr = OBJECTIVE_PATHCLEARANCE;
    } else if (boost::iequals("PathLength", objectiveStr)) {
        *objectivePtr = OBJECTIVE_PATHLENGTH;
    } else if (boost::iequals("ThresholdPathLength", objectiveStr)) {
        *objectivePtr = OBJECTIVE_THRESHOLDPATHLENGTH;
    } else if (boost::iequals("WeightedLengthAndClearanceCombo",
                              objectiveStr)) {
        *objectivePtr = OBJECTIVE_WEIGHTEDCOMBO;
    } else {
        std::cout << "Invalid objective string." << std::endl
                  << std::endl
                  << desc << std::endl;
        return false;
    }

    // Get the output file string and store it in the return pointer
    *outputFilePtr = vm["file"].as<std::string>();
}

#endif