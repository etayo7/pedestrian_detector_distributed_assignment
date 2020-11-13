# Distributed dynamic sensor assignment of multiple mobile targets
## Targets detected through object detection using deep learning

Given de number of targets of the problem and the id of the robot to compute the assignment, this workspace is able to:
- The detection node obtains frames from a RGB-D camera and at each frame tries to detect pedestrians through evaluating a YOLO detector.
- Knowing the colours of each target, previously initializated, is able to establish the id of each pedestrian detected and send to the assignment node the relative cost associated to that target and its relative coordinates.
- The assignment node reads the computed costs for the robot i coming from the detection node. Then, computes the assignment with a distributed simplex assignment algorithm and with the assignment obtained for the robot i to look target j, publish the desires relatives coordinates to the PTZ node 
- The PTZ node receives those coordinates and focus towards the target using a 2 axis movement with the camera.

(**Paper:** )

This ROS workspace holds some ROS packages that enable the next features:

* _**detection**_ : Detects the targets through the camera frames, computes the assignment costs and sets the relative position of the PTZ towards each target observed.
* _**simplex**_ : Computes the assignment in a distributed way of one of the robots known the costs and send to the PTZ the coordiantes of the assignment.
* _**PTZ_ROS**_ : Known the coordinates of the target, gives commands to the servos of the PTZ towards the target assigned. It also initializes the PTZ.
* _**Camera_ROS**_ : Initializes the RGB-D camera.

### Requirements:

* Ubuntu >=18.04
* ROS Melodic
* Python >=3.2
* OpenCV >=3.4.2

### ROS Dependencies:

* Eigen (**Master** version with 'all' indexing capabilities) 
* cv_bridge
* numpy
* dlib
* imutils
* cmake
* catkin
* librealsense2 (https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md)
* std_msgs
* usb-cam

### YOLO detector:
The YOLO detector used in this node uses a tiny implementation of the YOLO detector to be able to compute a real time detection on CPU. 
Yolo Darknet used already implemented in OpenCV >=3.4.2.
URL of the tiny weights and parameters used (already in this repository */detection/darknet*): https://pysource.com/2019/07/08/yolo-real-time-detection-on-cpu/

### Installation (ROS):

0. Install ROS Melodic and source it on the setup bashrc.sh file.
1. Clone the repository into your workspace src folder and source the workspace.
2. Install the ROS dependencies needed through ROS and pip installations.
3. In a terminal open the workspace folder, and run 'catkin_make'
4. If the compiler ask for some other dependencies, install it.
5. If the compiler do not find some .h files, try running 'catkin_make -j 1'.
6. When the compiler finishes, it's ready to use.

### Running:

(Previously to launch the nodes, make permission to the PTZ USB connection: *sudo chmod 666 /dev/ttyUSB0*)

#### Color initialization launcher:

_**color_initialization**_

To obtain the mean of the colours used by each target, later will be used to run the full launcher and differentiate each target.

Argument | Default | Description
------------ | ------------- | -------------
N | 3 | Number of targets

Example:

**roslaunch detection color_initialization.launch N:="3"**

#### Full launcher:

This launcher initializes the PTZ and the camera. Then, runs the detection node, the assignment node and the PTZ movement node.

_**assignment_face_detector**_

Argument | Default | Description
------------ | ------------- | -------------
robot | 1 | Id of the robot
N | 3 | Number of targets
costf | 0 | Relative cost = f(bounding box size=0) or f(central pixel depth=1)
pos_max | 1.5 | Absolute limit of the servo +-pos_max(rad)
colours | 92 227 101 67 123 95 80 179 122 | Colours of the targets [HSV mean] (HSV_1, HSV_2, HSV_3)

Example:

**roslaunch detection assignment_face_detector.launch robot:="2" N:="3" costf:="0" pos_max:="1.5" colours:="92 227 101 67 123 95 80 179 122"**


