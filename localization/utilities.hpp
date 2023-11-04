#ifndef MY_HEADER_HPP
#define MY_HEADER_HPP
#pragma once
#include <vector>
#include <cmath>

// Define the Point struct (assuming it's defined in "point.hpp")
#include "point.hpp"

using namespace std; // Add this line

// Function declarations
double mse(double x1, double x2, double y1, double y2);
vector<double> computeMSELoss(vector<Point> p);
void addscore(vector<Point> p);
void inc_age(vector<Point> p);
void del_nodes(vector<Point>& p, int threshold = 10);

#endif // MY_HEADER_HPP
