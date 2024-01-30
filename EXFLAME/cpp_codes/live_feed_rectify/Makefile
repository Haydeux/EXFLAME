CC=g++
inc_opencv=`pkg-config opencv4 --cflags --libs`
inc_pylon=-I/opt/pylon/include/ -L/opt/pylon/lib -lpylonbase -lpylonutility -lGenApi_gcc_v3_1_Basler_pylon -lGCBase_gcc_v3_1_Basler_pylon

all: live_feed_rectify

live_feed_rectify: live_feed_rectify.cpp
	$(CC) -o live_feed_rectify live_feed_rectify.cpp $(inc_opencv) $(inc_pylon)

.PHONY: clean
clean:
	rm -rf live_feed_rectify