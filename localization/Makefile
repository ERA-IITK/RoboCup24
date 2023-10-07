CC="g++"
PROJECT=localization
PROJECT2=depth_estimation
PROJECT3=heatmap
SRC=linedetection.cpp
SRC2=depth_estimation.cpp
SRC3=heatmap.cpp
LIBS=`pkg-config --libs opencv4 --cflags --libs`
# $(PROJECT): $(SRC)
# 	$(CC) $(SRC) -o $(PROJECT) $(LIBS)
$(PROJECT3): $(SRC3)
	$(CC) $(SRC3) -o $(PROJECT3) $(LIBS)
#export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/