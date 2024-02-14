# Baslers Class   
Author: Hayden Moxsom  
Date: 12th February 2024

### Description 
A semi-class based implementation of the ros_sender code. This program connects to two basler cameras, performs rectification, processes the image to extract a kiwifruit, performs stereo matching on this feature, and finally sends the position of the fruit in mm relative to the camera through a ROS message. 


# Prerequisites 
To compile and run this program, the following dependencies are required:  
 - g++ - Version 4:9.3.0-1ubuntu2 was used.  
 - [Threading Building Blocks (TBB)](https://en.wikipedia.org/wiki/Threading_Building_Blocks) - Version 2020.1-2 was used.
 - [OpenCV](https://opencv.org/) - Version 4.6.0 was used.
 - [Pylon Camera Software Suite](https://www2.baslerweb.com/en/downloads/software-downloads/) - Version 7.4.0 was used. 
 - [ROS](https://www.ros.org/) - Distribution [noetic](http://wiki.ros.org/noetic) was used.

### Installation Steps for Linux Ubuntu
For other systems, search the respective website for instructions.  

To install g++, run  the terminal command:
```bash
sudo apt install g++
```

To install TBB, run the terminal command:   
```bash 
sudo apt install libtbb-dev
```
If installing OpenCV on the Jetson, use the script available from [here (file + instructions)](https://forums.developer.nvidia.com/t/best-way-to-install-opencv-with-cuda-on-jetpack-5-xavier-nx-opencv-for-tegra/222777) or [here (source code)](https://github.com/AastaNV/JEP/blob/master/script/install_opencv4.6.0_Jetson.sh).  
For other devices, to install OpenCV run the following in a terminal for the basic install. This is from the [installation](https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html) page, see there for other options:
```bash 
sudo apt update && sudo apt install -y cmake g++ wget unzip
wget -O opencv.zip https://github.com/opencv/opencv/archive/4.x.zip
unzip opencv.zip
mkdir -p build && cd build
cmake ../opencv-4.x
cmake --build .
```
To install the Pylon Camera Software Suite download the correct installer for your system from the [pylon downloads](https://www2.baslerweb.com/en/downloads/software-downloads/) and follow the instructions (for the Jetson, get the ARM 64 version). The pylon libraries may need to be added to PATH. This can be done by adding `/opt/pylon/lib` to the `export LD_LIBRARY_PATH=` line in the `.bashrc` file. An example of this is shown below:
```sh
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/example/existing/lib:/opt/pylon/lib
```
To install ROS, follow the instructions available on the [installation page](http://wiki.ros.org/ROS/Installation). A summary of the commands is included here:
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl 
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
```

# Running
Before compiling this program the `DISPLAY_IMAGES` flag, located at the top of the program, can be changed. It is currently set to 1, but can be set to 0 or 1. Setting to 0 removes the image display, while setting to 1 allows the image display. No images runs faster, but is difficult for testing or debugging.  

To compile this program, navigate to the directory its located in and then run the commad:
```bash
make
```  

Once compiled, first launch the ROS master node in a seperate terminal using the command:
```bash
roscore
``` 

The baslers program can then be run from the same directoy as when compiling, using the terminal command:
```bash
./baslers_class
```
Or by specifiying the path to the executable (i.e. if its within the baslers_class folder it must be specified using `.../baslers_class/baslers_class`) e.g.:  
```bash 
~/path/to/executable/baslers_class
```  

To exit the program. If images are being displayed, select any image and press `Esc`. It no images are displayed, select the terminal that started the program and press `Ctrl + c` (this also works when images are displayed).