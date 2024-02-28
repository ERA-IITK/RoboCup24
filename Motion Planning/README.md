# Motion Planning for RoboCup MSL Robots

This folder includes the Webots simulation for motion planning (local and global) of a RoboCup MSL Robot using a 4-wheel omni-drive. This is maintained by Team ERA, IITK.

## Requirements

1. OpenCV v4.8.1 (C++ Version)
2. OMPL v1.6 (C++ Version, python bindings are not required)
2. Webots v2023b
3. cmake

## How to Run

- Install Webots and OpenCV
- Clone the repository and `% cd controllers/o1`
- Create a `build` directory and navigate to it: `% mkdir build && cd build`
- Configure using cmake with `% cmake ..`
- Build the project with `% make`
- Open the world in Webots and run the simulation
