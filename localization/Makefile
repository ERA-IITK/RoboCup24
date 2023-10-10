CXX = g++
CXXFLAGS = -std=c++11 -Wall

# List of source files and their corresponding targets
BUILD_DIR = build
SOURCES = linedetection.cpp depth_estimation.cpp heatmap.cpp rndgeneration.cpp
TARGETS = $(addprefix $(BUILD_DIR)/,$(SOURCES:.cpp=))
LIBS = `pkg-config --libs opencv4`
CFLAGS = `pkg-config --cflags opencv4`

# Rule to build all targets
all: $(TARGETS)

# Generic rule to build individual targets
$(BUILD_DIR)/%: %.cpp
	$(CXX) $(CXXFLAGS) $< -o $@ $(CFLAGS) $(LIBS)

# Rule to build testMain from testMain.cpp and its dependencies
$(BUILD_DIR)/hungarian: module_hungarian/Hungarian.cpp module_hungarian/Hungarian.h hungarian.cpp
	$(CXX) $(CXXFLAGS) $^ -o $@ $(CFLAGS) $(LIBS)

clean:
	rm -f $(TARGETS) $(BUILD_DIR)/testMain

#export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/