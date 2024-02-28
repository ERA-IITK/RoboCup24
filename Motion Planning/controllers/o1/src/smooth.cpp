// #include <iostream>
// #include <fstream>
// #include <vector>
// #include "spline.h"

// int main() {
//     // Read waypoints from the "output.txt" file
//     std::ifstream file("output.txt");
//     if (!file.is_open()) {
//         std::cerr << "Error opening file." << std::endl;
//         return 1;
//     }

//     std::vector<double> x_values, y_values;
//     double x, y;
//     while (file >> x >> y) {
//         x_values.push_back(x);
//         y_values.push_back(y);
//     }
//     file.close();

//     // Generate a smooth trajectory using Cubic Spline
//     tk::spline cs_x(x_values, y_values, tk::spline::cspline, true);
//     tk::spline cs_y(x_values, y_values, tk::spline::cspline, true);

//     // Array to store robot positions
//     std::vector<std::pair<double, double>> robot_positions;

//     // Calculate robot positions without running the animation
//     for (double frame = 0.0; frame < x_values.size() - 1; frame += 0.01) {
//         double x_robot = cs_x(frame);
//         double y_robot = cs_y(frame);
//         robot_positions.emplace_back(x_robot, y_robot);
//     }

//     // Display the array of robot positions
//     std::cout << "Robot Positions:\n";
//     for (const auto& pos : robot_positions) {
//         std::cout << "(" << pos.first << ", " << pos.second << ")\n";
//     }

//     // Write the array to the text file
//     std::ofstream output_file("output1.txt");
//     if (!output_file.is_open()) {
//         std::cerr << "Error opening output file." << std::endl;
//         return 1;
//     }

//     for (const auto& pos : robot_positions) {
//         output_file << pos.first << " " << pos.second << "\n";
//     }
//     output_file.close();

//     std::cout << "The array has been written to output1.txt\n";

//     return 0;
// }
