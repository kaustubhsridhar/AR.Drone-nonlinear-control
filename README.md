# AR.Drone nonlinear control (Gazebo Version)
This repository contains 3 ROS packages. 

Which have been used to implement backstepping control (as described in https://drive.google.com/open?id=0B3_gyQ1dIf-QdWE2S3BIQ3p1UVU) to make the AR Drone trace a particular trajectory in a Gazebo simulation.

Two have been downloaded - ardrone_autonomy and tum_simulator. 

One was created by me - ardrone_test.

## Description of contents of the 3 packages
### 1. ardrone_autonomy (downloaded)

this package connects the entire AR Drone system to the ROS environment giving you the control of the motion of the drone (by sunscribing to topics like /ardrone/takeoff, /ardrone/land, /cmd_vel to which you can publish takeoff, land and control commands resp.)

It also publishes the IMU, magnetometer and navigation (attitudes, velocities, processed accelerations, altitude, battery percent, etc.) data (through the /imu, /mag, /navdata and /navdata_gps topics) and camera images (through the /front/image_raw and /bottom/image_raw topics), etc. (http://ardrone-autonomy.readthedocs.io/en/latest/)

### 2. ardrone_test (created by me)

This package contains
1) src folder and CMakeLists.txt file
2) src/waypoint_nav.cpp file

Basically, this package contains a node (ardrone_test_node - with code in src/waypoint_nav.cpp file) that contains subscribers that subscribe to all the above topics published by the ardrone_autonomy node. This data (accelerometer, magnetometer, gyroscope, camera, gps, etc) can then be processed in this node. 

It also contains a subscriber that subscribes to /ground_truth/state to obtain pose and attitude values from ground observer in Gazebo.

This node also contains publishers that publish takeoff, land and drone control commands to /ardrone/takeoff, /ardrone/land and /cmd_vel topics. These publishers can be utilised through takeoff(), land() and move(lx,ly,lz,rx,ry,rz) fucntions. 

For simplicity and ease of use, running this node will open up a menu, where - 
(1) If you choose option m - it will allow you to press keyboard buttons to takeoff, land and perform basic movements (left, right, up, down, yaw). 
(2) If you choose option t (obviously after takeoff(option w)) - the nonlinear control will make it trace a predefined trajectory. (note: the code for control law implementation, predefined desired trajectory - can be found in function "traj_track()" in ardrone_test/src/waypoint_nav.cpp)

The below video shows an implementation of this code (notice the green trajectory (actual) tracking the red trajectory (desired))- 

(https://drive.google.com/open?id=0B3_gyQ1dIf-QUC05czBWVW90azQ)

### 3. tum_simulator (downloaded)

This package contains the implementation of a gazebo simulator for the Ardrone 2.0 . (http://wiki.ros.org/tum_simulator)

## Downloading/Installing the code
Run the following code lines in a terminal
```
mkdir catkin_ws/
```
```
cd catkin_ws/
```
```
git clone https://github.com/kaustubhsridhar/AR.Drone-nonlinear-control-Gazebo-Simulation.git
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
## Running a simulation of AR.Drone 2.0 traversing alond predefined trajectory (8 shape) on Gazebo
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
