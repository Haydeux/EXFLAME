# Overview
Title: Baslers Class  
Author: Hayden Moxsom  
Last Modified: 12/02/24 (February 12th)  
Description: A semi-class based implementation of the ros_sender code. This program connects to two basler cameras, performs rectification, processes the image to extract a kiwifruit, performs stereo matching on this feature, and finally sends the position of the fruit in mm relative to the camera through a ROS message. 

# Prerequisites 
To compile and run this program, the following dependencies are required:  
 - [Threading Building Blocks (TBB)](https://en.wikipedia.org/wiki/Threading_Building_Blocks)
 - [OpenCV](https://opencv.org/)
 - [Pylon Camera Software Suite](https://www2.baslerweb.com/en/downloads/software-downloads/) 
 - [ROS](https://www.ros.org/)  

### Installation Steps for Linux Ubuntu
For other systems, search the respective website for instructions.  
To install TBB, run the command:   
```bash 
sudo apt install libtbb-dev
```
To install OpenCV run the following for the basic install. This is from the [installation](https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html) page, see there for other options:
```bash 
sudo apt update && sudo apt install -y cmake g++ wget unzip
wget -O opencv.zip https://github.com/opencv/opencv/archive/4.x.zip
unzip opencv.zip
mkdir -p build && cd build
cmake ../opencv-4.x
cmake --build .
```
To install the Pylon Camera Software Suite download the correct installer for your system from the [pylon downloads](https://www2.baslerweb.com/en/downloads/software-downloads/) and follow the instructions (for the Jetson, get the ARM 64 version). The pylon libraries may need to be added to PATH. This can be done by adding `/opt/pylon/lib` to the `export LD_LIBRARY_PATH=` line in the `.bashrc` file. An example of this is shown below:
```
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/example/existing/lib:/opt/pylon/lib
```
To install ROS, follow the instructions available on the [installation page](http://wiki.ros.org/ROS/Installation). A summary of the commands is included here:
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl # if you haven't already installed curl
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
To compile this program, navigate to the directory its located in and then run the commad:
```bash
make
```
Once compiled it can be run from the same directoy using the command:
```bash
./baslers_class
```
Or by specifiying the path to the executable (i.e. if its within the baslers_class folder it must be specified using `.../baslers_class/baslers_class`) e.g.:
```bash 
~/path/to/executable/baslers_class
```
