cmake_minimum_required(VERSION 3.8)
project(localization)

set(LIBRARY_NAME libs)  # Adjust the library name as needed

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
add_library(${LIBRARY_NAME}
  src/rndgeneration.cpp
  src/linepoints.cpp
  src/linedetection.cpp
  src/realmap_location.cpp
  src/utilities.cpp
  src/LevenbergMarquardt.cpp
  #src/gradient_descent.cpp
)
add_executable(pipeline src/pipeline.cpp)

# Include directories for common headers
include_directories(include)

ament_target_dependencies(${LIBRARY_NAME}
  "cv_bridge"
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "OpenCV"
  "rcl_interfaces"
  "Eigen3"
  "ament_cmake"
  "nav_msgs"
  "tf2"
  "tf2_geometry_msgs"
)

target_include_directories(${LIBRARY_NAME} PUBLIC
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${ament_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${rcl_interfaces_INCLUDE_DIRS}
  Eigen3::Eigen
  ${nav_msgs_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(pipeline PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(pipeline PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_link_libraries(pipeline
  ${LIBRARY_NAME}  # Link against the library
  ${OpenCV_LIBRARIES}
  ${ament_LIBRARIES}
  ${cv_bridge_LIBRARIES}
  Eigen3::Eigen
)

install(TARGETS pipeline
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  include/
  DESTINATION include/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS)
  
ament_export_include_directories(include)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()