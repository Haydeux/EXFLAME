CC=gcc
inc_opencv=`pkg-config opencv4 --cflags --libs`
inc_pylon=-I/opt/pylon/include/ -L/opt/pylon/lib -lpylonbase -lpylonutility -lGenApi_gcc_v3_1_Basler_pylon -lGCBase_gcc_v3_1_Basler_pylon

all: improve_testing

improve_testing: improve_testing.cpp
	$(CC) -o improve_testing improve_testing.cpp -lm -lstdc++ -ltbb $(inc_opencv) $(inc_pylon)

.PHONY: clean
clean:
	rm -rf improve_testing