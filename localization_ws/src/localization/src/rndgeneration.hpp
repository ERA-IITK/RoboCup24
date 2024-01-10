#ifndef RANDOM_GENERATOR_HPP
#define RANDOM_GENERATOR_HPP
#pragma once
#include <iostream>
#include <random>
#include <vector>
#include "point.hpp"  // Include the header file containing the Point struct
#include "raw_points.hpp"
using namespace std;

// Function declarations
vector<Point> rndgen(int num_points);
vector<raw_pt> rndgen2(int num_points);

#endif // RANDOM_GENERATOR_HPP
