#ifndef RANDOM_GENERATOR_HPP
#define RANDOM_GENERATOR_HPP

#include <iostream>
#include <random>
#include <vector>
#include "point.hpp"  // Include the header file containing the Point struct

using namespace std;

// Function declarations
vector<Point> rndgen(int num_points);
vector<Point> callrndgen(int num);

#endif // RANDOM_GENERATOR_HPP
