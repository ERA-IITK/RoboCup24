cmake_minimum_required(VERSION 3.5)

# Setup the project. Its name is defined to be the controller directory name.
get_filename_component(PROJECT ${CMAKE_SOURCE_DIR} NAME)
project(${PROJECT})

# Set CPP Standards to C++20 for ensuring everything compiles correctly.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED YES)

# Get C or C++ sources in the src directory (only).
file(GLOB C_SOURCES src/*.c)
file(GLOB CPP_SOURCES src/*.cpp)
set(SOURCES ${C_SOURCES} ${CPP_SOURCES})
include_directories(include)

# Include OpenCV Library
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Include OMPL Package
find_package(ompl REQUIRED)
include_directories(${OMPL_INCLUDE_DIRS})

# Include BOOST Package Manually as the OMPL Import Scripts do not link all necessary components correctly.
find_package(Boost COMPONENTS program_options REQUIRED)

# Set the  Webots home path (change it according to your installation method)
if(APPLE)
    set(WEBOTS_HOME "/Applications/Webots.app/Contents")
    set(ENV{WEBOTS_HOME} "/Applications/Webots.app/Contents")
else()
    set(WEBOTS_HOME "/usr/local/webots")
    set(ENV{WEBOTS_HOME} "/usr/local/webots")
endif()

# Link with the Webots controller library.
link_directories($ENV{WEBOTS_HOME}/lib/controller)
set (LIBRARIES m ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX} ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX})
include_directories($ENV{WEBOTS_HOME}/include/controller/c $ENV{WEBOTS_HOME}/include/controller/cpp)

# Add the executables and Link Libraries.
if(APPLE)
    set(OMPL_LIBS
        /opt/homebrew/Cellar/ompl/1.6.0_3/lib/libompl.dylib
        # Add the following libraries if needed in the code.
        # /opt/homebrew/Cellar/flann/1.9.2_1/lib/libflann.dylib
        # /opt/homebrew/Cellar/ode/0.16.4/lib/libode.dylib
    )
    add_executable(${PROJECT} ${SOURCES})
    target_link_libraries(${PROJECT} ${LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} ${OMPL_LIBS})
else()
    add_executable(${PROJECT} ${SOURCES})
    target_link_libraries(${PROJECT} ${LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} ${OMPL_LIBRARIES})
endif()

# Copy the target executable at the right location.
add_custom_command(TARGET ${PROJECT} POST_BUILD COMMAND ${CMAKE_COMMAND} -E
  copy ${CMAKE_BINARY_DIR}/${PROJECT} ${CMAKE_SOURCE_DIR}
)
