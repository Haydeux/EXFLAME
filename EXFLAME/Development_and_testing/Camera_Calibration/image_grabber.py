from pypylon import pylon
import cv2 as cv



def scale_image(img, scale=100):
    width = int(img.shape[1] * scale / 100)
    height = int(img.shape[0] * scale / 100)
    dim = (width, height)
    img = cv.resize(img, dim, interpolation = cv.INTER_AREA)
    return img


def main():
    # Get all available devices
    devices = pylon.TlFactory.GetInstance().EnumerateDevices()

    if not devices:
        print("No cameras found.")
        return

    # Create and open the first camera
    camera1 = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateDevice(devices[0]))
    camera1.Open()

    # Create and open the second camera
    camera2 = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateDevice(devices[1]))
    camera2.Open()

    try:
        # Set camera parameters for camera1 if needed
        # camera1.ExposureTime.SetValue(10000)

        # Set camera parameters for camera2 if needed
        # camera2.ExposureTime.SetValue(10000)

        # Start grabbing images for camera1
        camera1.StartGrabbing()

        # Start grabbing images for camera2
        camera2.StartGrabbing()


        counter = 1 
        key = 0

        while camera1.IsGrabbing() and camera2.IsGrabbing() and key != 27:
            # Grab and retrieve images from camera1
            grab_result1 = camera1.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
            if grab_result1.GrabSucceeded():
                image1 = grab_result1.Array
                # Process image from camera1 as needed
                # ...

            grab_result1.Release()

            # Grab and retrieve images from camera2
            grab_result2 = camera2.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
            if grab_result2.GrabSucceeded():
                image2 = grab_result2.Array
                # Process image from camera2 as needed
                # ...

            grab_result2.Release()

            
            image1 = cv.cvtColor(image1, cv.COLOR_BayerBG2RGB)
            image2 = cv.cvtColor(image2, cv.COLOR_BayerBG2RGB)
            # image1 = cv.cvtColor(image1, cv.COLOR_RGB2BGR)
            # image2 = cv.cvtColor(image2, cv.COLOR_RGB2BGR)

            cv.imshow("Left", image1)
            cv.imshow("Right", image2)
            key = cv.waitKey(1)

            if key == 115:
                print("saving")
                cv.imwrite(f"Images/left/left_image_{counter}.png",image1)
                cv.imwrite(f"Images/right/right_image_{counter}.png",image2)
                counter += 1

    finally:
        # Stop grabbing images for both cameras
        camera1.StopGrabbing()
        camera2.StopGrabbing()

        # Close both cameras
        camera1.Close()
        camera2.Close()

        cv.destroyAllWindows()

if __name__ == "__main__":
    main()