# Distributed dynamic sensor assignment of multiple mobile targets

## Installation guide

### Pre-requisites

- **Ubuntu** [18.04](https://releases.ubuntu.com/18.04/)*(tested)* or newer *(on your own risk)*
- **ROS** [Melodic](https://wiki.ros.org/melodic/Installation/Ubuntu)
  - **Python** 2.7
- **Python** 3.2 or newer :arrow_right: `sudo apt-get install python3`

- **OpenCV** 3.4.2 :arrow_right: `pip install -I opencv==3.4.2.16` then same but with `pip3`

### ROS Dependencies

- eigen (Added already as a *git submodule* from [here](https://gitlab.com/libeigen/eigen). The module should be at commit `2627e2f2`. Use `git checkout 2627e2f2e6125cf09fa32789755135e84552275b`on it if is on a different commit)
  *Original author indicates another commit as the right one:* [SHA hash - 70fbcf82ed0a95b27ee68e20199a4e8e1e913268]
- cv_bridge: `sudo apt-get install ros-melodic-cv-bridge`
- numpy: `pip install numpy` & `pip3 install numpy` *(Skip if you already installed OpenCV 3.4.2)* 
- dlib: `pip install dlib` & `pip3 install dlib`
- imutils: `pip install imutils` & `pip3 install imutils`
- cmake: `sudo apt-get install cmake`
- catkin: `sudo apt-get install catkin`
- std_msgs `sudo apt-get install ros-melodic-std-msgs`
- usb-cam `sudo apt-get install ros-melodic-usb-cam`
- realsense2-camera:`sudo apt-get install ros-melodic-realsense2-camera`

### YOLO detector

The YOLO Darknet used is implemented in OpenCV 3.4.2 and newer. For the parameters used check [here](https://pysource.com/2019/07/08/yolo-real-time-detection-on-cpu/). They are also found in `{repo}/detection/darknet`.

### Installation steps

1. Install Ubuntu using preffered method.
2. Install ROS Melodic, preferably the full-desktop type: `sudo apt install ros-melodic-desktop-full`. Follow the link in *pre-requisites* for more details.
3. Install Python 3 and OpenCV using the commands given above.
4. Install ROS packages using the commands given above.

5. Create a catkin workspace (guide [here](https://wiki.ros.org/catkin/Tutorials/create_a_workspace)) and clone the repository into the `{workspace_path}/src` sub-folder.
6. Open a terminal in the workspace root folder and run `catkin_make`.
   1. If that returns errors, run `catkin_make simplex_generate_messages` and if that is successful, run `catkin_make` as normal afterwards.
   2. If the compiler asks for any dependencies, install them.
   3. If the compiler does not find some `.h` files, try running `catkin_make -j 1` or `catkin_make_isolated` instead of the normal command.
7. Source everything in the `.bashrc` file, run the following commands:`
   1. Source ROS: `echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc`
   2. Source the workspace: `echo "source {workspace_path}/devel/setup.bash" >> ~/.bashrc`
   3. Update current terminal session settings: `source ~/.bashrc`

### Test functionality

(Using turtlebots and Intel RealSense cameras)

#### Camera functionality

Connect the camera to the PC via a USB 3 port. If working on a VM, connect the camera to the VM instead of the host OS. In a terminal, run `lsusb` and check for an Intel RealSense device.

Run `ls -l /dev/` and identify the camera there. It should look like `ttyUSB0`. Change the rights to the camera with `sudo chmod 666 /dev/{camera_identifier}`.

Run `realsense-viewer`. Make sure that the camera is detected, turn on the <u>Stereo Module</u> and the <u>RGB Camera</u> and if everything displays right, the camera itself is working properly.

To test if it works with ROS:

- start the OS (run `roscore` and leave it in the background)
- start a node that captures camera data: `roslaunch realsense2_camera rs_camera.launch`
- run the visualizer tool: `rviz`
- In the visualizer tool
  -  in the left panel set the *Fixed Frame* to `camera_depth_frame`
  - Click on the lower left button `Add - By topic - /camera/color/image_raw - Image` and click OK
  - Repeat and add `/camera/depth/image_rect_raw - DepthCloud` and click OK

If rviz shows both the image and the 3D projection, then everything is working fine.

#### Detection package

###### Color initialization

To initialize the detection module with the colors of the targets, use:

`roslaunch detection color_initialization.launch N:="<no. of targets>"`

Optionally add the parameter `initial_reset:=true` to reset the camera at launch and prevent hardware errors.

Running this should return the HSV-style median color of each target, which can then be used in the assignment problem.