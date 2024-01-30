# Testing seg fault in python

from pypylon import pylon


def main():    
    print("Program starting")

    tlf = pylon.TlFactory.GetInstance()
    devices = tlf.EnumerateDevices()
    
    # -- ISSUE -- 
    camera_left = pylon.InstantCamera(tlf.CreateDevice(devices[0]))
    camera_right = pylon.InstantCamera(tlf.CreateDevice(devices[1]))
    # -----------

    print("Program ending")
            

if __name__ == "__main__":
    main()


# Has similar issue to C++ code. Except InstantCamera causes a "terminate called without an active exception" here.
# Possibly something to do with it starting a C++ thread to handle the cameras, with the 
# thread unable to join correctly due to a corrupted stack and return address