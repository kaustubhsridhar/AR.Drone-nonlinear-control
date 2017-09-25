# AR.Drone nonlinear control (Gazebo Version)
This repository contains 4 ROS packages. 

Three have been downloaded - ardrone_autonomy, tum_ardrone and tum_simulator. 

One was created by me - ardrone_test.

## Description of contents of 4 packages
### 1. ardrone_autonomy (downloaded)

this package connects the entire AR Drone system to the ROS environment giving you the control of the motion of the drone (by sunscribing to topics like /ardrone/takeoff, /ardrone/land, /cmd_vel to which you can publish takeoff, land and control commands resp.)

It also publishes the IMU, magnetometer and navigation (attitudes, velocities, processed accelerations, altitude, battery percent, etc.) data (through the /imu, /mag, /navdata and /navdata_gps topics) and camera images (through the /front/image_raw and /bottom/image_raw topics), etc. (http://ardrone-autonomy.readthedocs.io/en/latest/)

### 2. ardrone_test (created by me)

This package cpntains node (ardrone_test_node - with code in waypoint_nav.cpp file) that contains subscribers that subscribe to all the above topics published by the ardrone_autonomy node. This data (accelerometer, magnetometer, gyroscope, camera, gps, etc) can then be processed in this node. 

It also contains a subscriber that subscribes to /ground_truth/state to obtain pose and attitude values from ground observer in Gazebo.

This node also contains publishers that publish takeoff, land and drone control commands to /ardrone/takeoff, /ardrone/land and /cmd_vel topics. These publishers can be utilised through takeoff(), land() and move(lx,ly,lz,rx,ry,rz) fucntions. 

For simplicity and ease of use, running this node will open up a menu, where - 
(1) If you choose option m - it will allow you to press keyboard buttons to takeoff, land and perform basic movements (left, right, up, down, yaw). 
(2) If you choose option t (obviously after takeoff(option w)) - the nonlinear control will make it trace a predefined trajectory. (note: the code for control law implementation, predefined desired trajectory - can be found in function "traj_track()" in ardrone_test/src/waypoint_nav.cpp

(https://drive.google.com/open?id=0B3_gyQ1dIf-QTmE5Z200Y3JmVGc)

### 3. tum_ardrone [used only drone_gui node] (downloaded)

Running this node will open a simple GUI for controlling the drone (i.e. move up, down, left, right, takeoff and land). This GUI is an alternate option to perform basic movements and to land in case of emergencies. (http://wiki.ros.org/tum_ardrone/drone_gui)

### 4. tum_simulator (downloaded)

This package contains the implementation of a gazebo simulator for the Ardrone 2.0 . (http://wiki.ros.org/tum_simulator)

## Builidng the code (after downloading into catkin_ws folder in home directory)
Run the following code lines in different terminals
```
cd catkin_ws/
```
```
catkin_make
```
## Running the code on the actual AR.Drone 2.0
Run the following code lines in different terminals

```
roscore
```
```
rosrun ardrone_autonomy ardrone_driver _navdata_demo:=0
```
```
rosrun ardrone_test ardrone_test_node
```
```
rosrun tum_ardrone drone_gui
```
## Running a simulation of AR.Drone 2.0 on Gazebo
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
```
rosrun tum_ardrone drone_gui
```
