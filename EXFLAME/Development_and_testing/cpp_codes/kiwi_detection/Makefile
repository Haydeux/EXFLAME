CC=g++
inc_opencv=`pkg-config opencv4 --cflags --libs`
inc_pylon=-I/opt/pylon/include/ -L/opt/pylon/lib -lpylonbase -lpylonutility -lGenApi_gcc_v3_1_Basler_pylon -lGCBase_gcc_v3_1_Basler_pylon

all: kiwi_detection

kiwi_detection: kiwi_detection.cpp
	$(CC) -o kiwi_detection kiwi_detection.cpp -ltbb $(inc_opencv) $(inc_pylon)

.PHONY: clean
clean:
	rm -rf kiwi_detection