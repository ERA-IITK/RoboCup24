CC="g++"
PROJECT=localization
PROJECT1=linedetection
PROJECT2=depth_estimation
PROJECT3=heatmap
PROJECT4=rndgeneration
SRC1=linedetection.cpp
SRC2=depth_estimation.cpp
SRC3=heatmap.cpp
SRC4=rndgeneration.cpp
LIBS=`pkg-config --libs opencv4 --cflags --libs`
# $(PROJECT): $(SRC)
# 	$(CC) $(SRC) -o $(PROJECT) $(LIBS)
$(PROJECT4): $(SRC4)
	$(CC) $(SRC4) -o $(PROJECT4) $(LIBS)
#export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/