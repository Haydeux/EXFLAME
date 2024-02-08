CC=g++
inc_opencv= $(shell pkg-config opencv4 --cflags --libs)
inc_pylon= -I/opt/pylon/include/ -L/opt/pylon/lib -lpylonbase -lpylonutility -lGenApi_gcc_v3_1_Basler_pylon -lGCBase_gcc_v3_1_Basler_pylon
inc_ros= $(shell pkg-config roscpp --cflags --libs) $(shell pkg-config std_msgs --cflags --libs) $(shell pkg-config geometry_msgs --cflags --libs)

all: baslers_class

baslers_class: baslers_class.cpp
	$(CC) -O2 -o baslers_class baslers_class.cpp -lm -lstdc++ -ltbb $(inc_opencv) $(inc_pylon) $(inc_ros)

.PHONY: clean
clean:
	rm -rf baslers_class