CC="g++"
PROJECT=localization
SRC=linedetection.cpp

LIBS=`pkg-config --libs opencv4 --cflags --libs`
$(PROJECT): $(SRC)
	$(CC) $(SRC) -o $(PROJECT) $(LIBS)

#export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/