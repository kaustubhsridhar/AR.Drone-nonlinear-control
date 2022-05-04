# AR.Drone nonlinear control 
This repository contains 4 ROS packages. 

Which have been used to implement backstepping control (as described in [this paper](https://drive.google.com/file/d/0B3_gyQ1dIf-QdWE2S3BIQ3p1UVU/view?usp=drivesdk&resourcekey=0-ZZI9-CjLFfEgdg7H56zr8g)) to make the AR Drone trace a particular trajectory in a Gazebo simulation and in real life (with a VICON Motion Capture system) It also uses the same controller to follow an RC Car.

## Description of contents of the 4 packages
### 1. ardrone_autonomy (downloaded)

this package connects the entire AR Drone system to the ROS environment giving you the control of the motion of the drone (by sunscribing to topics like /ardrone/takeoff, /ardrone/land, /cmd_vel to which you can publish takeoff, land and control commands resp.)

It also publishes the IMU, magnetometer and navigation (attitudes, velocities, processed accelerations, altitude, battery percent, etc.) data (through the /imu, /mag, /navdata and /navdata_gps topics) and camera images (through the /front/image_raw and /bottom/image_raw topics), etc. (http://ardrone-autonomy.readthedocs.io/en/latest/)

### 2. ardrone_test (created by me)

This package contains
1) src folder and CMakeLists.txt file
2) src/waypoint_nav.cpp file

Basically, this package contains the ardrone_test_node node (with code in src/waypoint_nav.cpp file) that contains subscribers that subscribe to all the above topics published by the ardrone_autonomy node. This data (accelerometer, magnetometer, gyroscope, camera, gps, etc) can then be processed in this node. 

It also contains a subscriber that subscribes to /ground_truth/state to obtain pose and attitude values from ground observer in Gazebo.

This node also contains publishers that publish takeoff, land and drone control commands to /ardrone/takeoff, /ardrone/land and /cmd_vel topics. These publishers can be utilised through takeoff(), land() and move(lx,ly,lz,rx,ry,rz) fucntions. 

For simplicity and ease of use, running this node will open up a menu, where - 
(1) If you choose option m - it will allow you to press keyboard buttons to takeoff, land and perform basic movements (left, right, up, down, yaw). 
(2) If you choose option t (obviously after takeoff(option w)) - the nonlinear control will make it trace a predefined trajectory in Gazebo. (note: the code for control law implementation, predefined desired trajectory - can be found in function "traj_track()" in ardrone_test/src/waypoint_nav.cpp)
(3) If you choose option v (obviously after takeoff(option w)) - the nonlinear control will make it trace a predefined trajectory IRL using VICON's help. (note: the code for control law implementation, predefined desired trajectory - can be found in function "traj_track()" in ardrone_test/src/waypoint_nav.cpp)

![Alt text](https://raw.github.com/kaustubhsridhar/AR.Drone-nonlinear-control/master/menu_img.png "menu shown on running ardrone_test_node node")

#### Click on the below image to see a video that shows an implementation of this code in a Gazebo simulation (notice the green trajectory (actual) tracking the red trajectory (desired))- 

<a href="http://www.youtube.com/watch?feature=player_embedded&v=Bcu3NSuPaPY" target="_blank"><img src="http://img.youtube.com/vi/Bcu3NSuPaPY/0.jpg" alt="Gazebo Simulation of Backstepping Control on Parrot AR.Drone 2.0" width="480" height="360" border="0" /></a>

#### Click on the below image to see a video that shows a real life implementation of the AR Drone tracking an 8 shape trajectory (using VICON)

<a href="http://www.youtube.com/watch?feature=player_embedded&v=DCzn9Angy2s" target="_blank"><img src="http://img.youtube.com/vi/DCzn9Angy2s/0.jpg" alt="Real life implementation of Backstepping Control on Parrot AR.Drone 2.0" width="480" height="360" border="0" /></a>

#### Click on the below image to see a video that shows the AR Drone following an RC Car using the same controller (using VICON)

<a href="http://www.youtube.com/watch?feature=player_embedded&v=riiJ3pi4EY8" target="_blank"><img src="http://img.youtube.com/vi/riiJ3pi4EY8/0.jpg" alt="Real life implementation of Backstepping Control on Parrot AR.Drone 2.0" width="480" height="360" border="0" /></a>

### 3. tum_simulator (downloaded)

This package contains the implementation of a gazebo simulator for the Ardrone 2.0 . (http://wiki.ros.org/tum_simulator)

### 4. vicon_bridge (downloaded)

This is a driver providing data from VICON motion capture system.

## Downloading/Installing the code
Run the following code lines in a terminal
```
mkdir catkin_ws/
```
```
cd catkin_ws/
```
```
git clone https://github.com/kaustubhsridhar/AR.Drone-nonlinear-control.git
```
(note that the src folder should be directly in the catkin_ws folder. delete any unnecessary folder created by git cloning)

## Builidng the code 
Run the following code lines in a terminal
```
cd catkin_ws/
```
```
catkin_make
```
Run the below line only once (after the first compilation). It adds the workspace to the search path.
```
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc 
```
## Running a simulation of AR.Drone 2.0 traversing along predefined trajectory (8 shape) on Gazebo
Run the following code lines in different terminals

```
roscore
```
```
roslaunch cvg_sim_gazebo ardrone_testworld.launch
```
```
rosrun ardrone_test ardrone_test_node
```
Now, press w to takeoff
```
rosrun ardrone_test ardrone_test_node
```
Press t to start trajectory tracking. (data of desired and actual positions can be saved and plotted)

## Making an AR.Drone 2.0 traverse along a predefined trajectory (8 shape) IRL with the help of Vicon Motion Capture system
Run the following code lines in different terminals
Make sure your computer is connected to same LAN as Vicon's rig and wifi is connected to ardrone

```
roscore
```
```
roslaunch vicon_bridge vicon.launch
```
```
rosrun ardrone_autonomy ardrone_driver
```
```
rosrun ardrone_test ardrone_test_node
```
Now, press w to takeoff
```
rosrun ardrone_test ardrone_test_node
```
Press v to start trajectory tracking. (data of desired and actual positions can be saved and plotted)
