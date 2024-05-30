#pragma once
#ifndef LINEPOINTS_HPP
#define LINEPOINTS_HPP
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include "point.hpp" 
#include "linepoint.hpp"

// Define the line struct
struct line {
    float a, b, c;
    // assuming the line is of type ax + by = c
};

// Define the limits struct
struct limits {
    float xmin, xmax, ymin, ymax;
};

// Function declarations
bool CompareByD(const linepoint &a, const linepoint &b);
float distance(float x1, float y1, float x2, float y2);
// void solve_for_centreCircle(WPoint p, float theta, std::vector<linepoint> temp);
// void solve_for_quad1(WPoint p, float theta, std::vector<linepoint> temp);
// void solve_for_quad2(WPoint p, float theta, std::vector<linepoint> temp);
// void solve_for_quad3(WPoint p, float theta, std::vector<linepoint> temp);
// void solve_for_quad4(WPoint p, float theta, std::vector<linepoint> temp);
linepoint findNearestPoint(WPoint a);

#endif // LINEPOINTS_HPP
