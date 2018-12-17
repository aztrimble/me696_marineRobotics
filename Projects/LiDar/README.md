# LiDar with Velodyne VLP16

Created by: Brenden Minei
Last update: 2018.12.16

In this work, we will explore different methods for robot laser odometry and mapping (LOAM) using ROS and Velodyne VLP16 lidar.  Laser Odometry and Mapping (Loam) is a realtime method for state estimation and mapping using a 3D lidar. The program contains two major threads running in parallel. An "odometry" thread computes motion of the lidar between two sweeps, at a higher frame rate. It also removes distortion in the point cloud caused by motion of the lidar. A "mapping" thread takes the undistorted point cloud and incrementally builds a map, while simultaneously computes pose of the lidar on the map at a lower frame rate. The lidar state estimation is combination of the outputs from the two threads.

If an IMU is available, the orientation (integrated from angular rate) and acceleration measurements are used to deal with general motion of the lidar, while the program takes care of the linear motion.

## Directory
  - ['Frank_Moosmann'](Frank_Moosmann): LOAM using SVMs for object identification with dynamic tracking and course prediction
 - ['LeGO-LOAM'](LeGO-LOAM): LOAM based off of loam_velodyne with the addition of surface and edge detection for additional filtering
 - ['Literature'](Literature): Literature on Lidar LOAM
 - ['loam_velodyne'](loam_velodyne): Basic LOAM with particle filtering
 - ['Velodyne_Resources'](Velodyne_Resources): Manuals for Velodyne VLP16

 - ['Bag Files'](http://www.frc.ri.cmu.edu/~jizhang03/Datasets/): Additional bag files can be found here

## Installing and Running Velodyne Drivers

Instructions on how to setup the Velodyne VLP16 and install the drivers can be found [here](http://wiki.ros.org/velodyne/Tutorials/Getting%20Started%20with%20the%20Velodyne%20VLP16).

Information on Velodyne Nodes and Nodelets can be found [here](http://wiki.ros.org/velodyne_pointcloud).

## loam-velodyne (Tested)
### Installation

Download or clone the loam-velodyne folder into ```$ cd ~/catkin_ws/src```.  Then:
```
$ cd ~/catkin_ws
$ catkin_make -DCMAKE_BUILD_TYPE=Release
```

### Running

Launch ROS in first terminal:
```
source ~/catkin_ws/devel/setup.bash
roslaunch loam_velodyne loam_velodyne.launch
```

In second terminal play bag file:
```
rosbag play ~/Downloads/velodyne.bag
```
Or to ensure imu and clock sync are used (usually available by default):
```
rosbag play *.bag --clock --topic /velodyne_points /imu/data
```
Or read from velodyne pcap:
```
roslaunch velodyne_pointcloud VLP16_points.launch pcap:="/home/account_name/Downloads/velodyne.pcap"
```
To use live data:
```
roslaunch velodyne_pointcloud VLP16_points.launch
```

__Important Notes:__  IMU should be oriented with x-axis forward to align with VLP16 or must be transposed.  IMU should also be placed as close as possible to lidar.  Takes in messages from /imu/data.  Can run without imu if rotational degrees of freedom of sensor are limited/constrained.

### Pros
 - Provides detailed point cloud with maximum sensor range
 - Requires only two threads to run without rviz
 - Reduces data usage by dumping map data after every 50 meters

### Cons
 - Will not automatically dump points from dynamic objects
 - Must have reference points or position will diverge (axises will rotate from covariance)
 - Will not save full point cloud map history

### Resources
 - https://github.com/laboshinl/loam_velodyne
 - http://www.frc.ri.cmu.edu/~jizhang03/

## LeGO-LOAM (Tested)
### Installation

First download gtsam from ['here'](https://bitbucket.org/gtborg/gtsam/downloads/).  Then install gtsam with the downloaded file in same directory:
```
$ mkdir build
$ cd build
$ cmake .. ~/name of file downloaded
$ sudo make install
```
Download or clone the LeGO-LOAM folder into ```$ cd ~/catkin_ws/src```.  Then:
```
$ cd ~/catkin_ws
$ catkin_make -j1
```
When you compile the code for the first time, you need to add "-j1" behind "catkin_make" for generating some message types. "-j1" is not needed for future compiling.

### Running

Code will run with VLP16 by default, but key parameters can be changed for new sensor type, physical sensor offsets (i.e. mount anlge), and/or point cloud filter tuning under file ```/src/LeGO-LOAM/LeGO-LOAM/include/utility.h```.  May need to redo catkin_make if changed.

In first termincal launch ROS:
```
source ~/catkin_ws/devel/setup.bash
roslaunch lego_loam run.launch
```
In second terminal play bag file:
```
rosbag play *.bag --clock --topic /velodyne_points /imu/data
```
To play live data:
```
roslaunch velodyne_pointcloud VLP16_points.launch
```
__Notes:__ Under ```/src/LeGO-LOAM/LeGO-LOAM/launch/run.launch``` the parameter "/use_sim_time" is set to "true" for simulation, "false" to real robot usage.  May need to rebuild after change.

__Important Notes:__  IMU should be oriented with x-axis forward to align with VLP16 or must be transposed.  IMU should also be placed as close as possible to lidar.  Takes in messages from /imu/data.  Can run without imu if rotational degrees of freedom of sensor are limited/constrained.

### Pros
 - Same as loam-velodyne with additional filtering with surface and edge detection
 - Position history on rviz
 - Cleaner point cloud generated with predetermine point-to-point spacing
 - Does not dump data and keeps full point cloud history

### Cons
 - Range is reduced without translational movement for scan validation of point to register
 - Reduced details with additional filtering
 - Requires ground reference to keep position tracking (may be corrected using gps and second imu odom)
 - May not dump points created from dynamic objects (not tested or verified)

### Resources
 - https://github.com/RobustFieldAutonomyLab/LeGO-LOAM
 - https://bitbucket.org/gtborg/gtsam/overview

## Frank_Moosmann (Untested)

The software tools provided can be used for Simultaneous Localization and Mapping (SLAM) with Detection And Tracking of Moving Objects (DATMO).
A sequence of range images serves as input, hence arbitrary dense range sensors can be used.
However, testing was only performed on data from the Velodyne HDL-64E S2.  This utilizes SVMs to identify objects.  The identified objects can be determined as static or dynamic and be tracked or its movement predicted (i.e. predict crash).  The code is in C++.

### Resources
 - http://www.mrt.kit.edu/z/publ/download/velodynetracking/code.html

## Google Cartographer (Untested)

Cartographer is a system that provides real-time simultaneous localization and mapping (SLAM) in 2D and 3D across multiple platforms and sensor configurations. This project provides Cartographer's ROS integration.  Requires 2 VLP16's to run.

### Resources
 - https://github.com/googlecartographer/cartographer_ros

## nu_jackal_autonav

Built on the same platform Clearpath Jackal UGV as LeGO-LOAM.  Uses existing gmapping, hector_slam, AMCL, global_planner, and NavFN.  This could simplify lidar with autonomous functions using existing ros libraries.

### Resources
 - https://github.com/njkaiser/nu_jackal_autonav

## LiDAR-based Collision Avoidance

Collision aviodance using VLP16 for DJI drones.

### Resources
 - https://developer.dji.com/onboard-sdk/documentation/modules/collision-avoidance/collision-avoidance.html

## VLOAM

This is another implentation from loam-velodyne on a KITTI.

### Resources
- https://github.com/stevenliu216/568-Final-Project

## Lidar camera calibration

The package is used to calibrate a Velodyne LiDAR with a camera (works for both monocular and stereo). Specficially, Point Gray Blackfly and ZED camera have been successfully calibrated against Velodyne VLP-16 using lidar_camera_calibration. It shows the accuracy of the proposed pipeline by fusing point clouds, with near perfection, from multiple cameras kept in various positions.

The package finds a rotation and translation that transform all the points in the LiDAR frame to the (monocular) camera frame. 

### Resources
 - https://github.com/ankitdhall/lidar_camera_calibration


