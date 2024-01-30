CC=g++
inc_pylon= -I/opt/pylon/include/ -L/opt/pylon/lib -lpylonbase -lpylonutility -lGenApi_gcc_v3_1_Basler_pylon -lGCBase_gcc_v3_1_Basler_pylon

all: test_seg_fault

test_seg_fault: test_seg_fault.cpp
	$(CC) -g -o test_seg_fault test_seg_fault.cpp $(inc_pylon)

.PHONY: clean
clean:
	rm -rf test_seg_fault