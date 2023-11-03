CXX = g++
CXXFLAGS = -g -std=c++11 -Wall

BUILD_DIR = build

SOURCES = rndgeneration.cpp linepoints.cpp realmap_location.cpp utilities.cpp pipeline.cpp gradient_descent.cpp
TARGETS = $(addprefix $(BUILD_DIR)/,$(SOURCES:.cpp=.o))
EXECUTABLE = $(BUILD_DIR)/pipeline

LIBS = `pkg-config --libs opencv4`
CFLAGS = `pkg-config --cflags opencv4`

$(EXECUTABLE): $(TARGETS)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(CFLAGS) $(LIBS)

$(BUILD_DIR)/%.o: %.cpp %.hpp point.hpp linepoint.hpp raw_points.hpp
	$(CXX) $(CXXFLAGS) -c $< -o $@ $(CFLAGS)

all: $(EXECUTABLE)
# $(BUILD_DIR)/gradient_descent.o: gradient_descent.cpp gradient_descent.hpp point.hpp linepoint.hpp raw_points.hpp linepoints.o
# 	$(CXX) $(CXXFLAGS) -c $< -o $@ $(CFLAGS)
$(BUILD_DIR)/pipeline.o: pipeline.cpp point.hpp linepoint.hpp raw_points.hpp
	$(CXX) $(CXXFLAGS) -c $< -o $@ $(CFLAGS)
clean:
	rm -f $(TARGETS) $(EXECUTABLE)
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/