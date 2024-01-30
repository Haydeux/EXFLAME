// Testing the segmentaion fault caused by basler cameras

#include <pylon/PylonIncludes.h>
using namespace Pylon;

#include <iostream>

int main(int argc, char **argv) {
    std::cout << "Program starting" << std::endl;
    PylonInitialize();

    // Get all available camera devices
    CTlFactory& tlFactory = CTlFactory::GetInstance();
    DeviceInfoList_t devices;
    tlFactory.EnumerateDevices(devices);


    // Cause of issue 
    // --------------------------------------------------

    // Connecting to one gives a bus error at program end
    //CInstantCamera camera(CTlFactory::GetInstance().CreateDevice(devices[0]));

    // Connecting to both gives a segmentation fault at program end
    // CInstantCamera camera_left(CTlFactory::GetInstance().CreateDevice(devices[0]));
    // CInstantCamera camera_right(CTlFactory::GetInstance().CreateDevice(devices[1]));

    // Exact same issue as above 2 cases. 1 camera is bus error, 2 cameras is segmentation fault
    CInstantCameraArray cameras(devices.size());
    for (size_t i = 0; i < cameras.GetSize(); ++i) {
        cameras[i].Attach(tlFactory.CreateDevice(devices[i]));
    }

    // --------------------------------------------------


    PylonTerminate(true);

    std::cout << "Program Ended" << std::endl;
    // exit(0); // Exits the program without issue, as it does not need the return address thats on the stack 

    return 0; // Issue occurs here, when trying to access the memory stored at the return address
}

    // -- FINDINGS --   
    // Likely that stack becomes corrupted somehow by the CInstantCamera class. This causes an invalid return address
    // Bus error from 1 camera -> memory address alignment is incorrect or out of bounds (memory does not exist)
    // Segmentation fault from 2 cameras -> return adrress points to read only memory or similar (invalid access to memory that exists)
    // No idication that anyone else has encountered this specific problem anywhere on google

    // Tried a different device running the same program. No issues occur


    // This indicates the problem is either exclusive to the jetson devices, or this jetson in particular.
    // Could be due to jetson not fully supporting this pylon version, or the pylon software was not correctly installed onto the jetson. 


    // -- CONCLUSION -- 
    // Unable to find fix to issue. Can workaround this, but may not be ideal, as this 
    // requires it to be the only function (as it will be unable to return from it).

    // Will use "exit(0)" to end the program instead. 
    