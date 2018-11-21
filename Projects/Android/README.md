# me696_marineRobotics
Repository for the ME 696 - Marine Robotics and ROS course

## Android Modules
### 1. Directory listing
 - [`Kanaloa Camera One`](</Android Modules/android_kanaloa_camera_one>): Kanaloa Camera One Android App Source
 - [`Kanaloa Camera Two`](</Android Modules/android_kanaloa_camera_two>): Kanaloa Camera Two Android App Source
 - [`Kanaloa Camera Three`](</Android Modules/android_kanaloa_camera_three>): Kanaloa Camera Three Android App Source
 - [`Kanaloa Camera Four`](</Android Modules/android_kanaloa_camera_four>): Kanaloa Camera Four Android App Source
 - [`Kanaloa Location`](</Android Modules/android_kanaloa_location>): Kanaloa imu and gps location Android App Source
 
### 2. Setup ROS and Android studio on Ubuntu
  - First Download and set up Android Studio on your ubunutu machine. 
      * Follow this tutorial [Android Studio & Tools Download](http://wiki.ros.org/android/kinetic/Android%20Studio/Download)
  - Then setup your ROS environment for development.
      * Follow this tutorial [Installation - ROS Development Environment](http://wiki.ros.org/android/Tutorials/kinetic/Installation%20-%20ROS%20Development%20Environment)
      * only need to do step 3.1
          + For the first command, it might be better to do ```mkdir -p ~/android_ros_workspace```, rather name the directory "android_core"
          + Be sure to run ```catkin_make``` before moving on to next steps
  
### 3. Importing Kanaloa Android App modules
  - Download/clone this repo to a folder on your computer.
  - Switch to branch ```tyson```
  - Import Module ```File -> New -> Import Module```
  ![Import module](https://i.stack.imgur.com/Nlpfo.png)
  - Find and Select the android_kanaloa_* module you want to import
  - Press the Sync Gradle project ```File -> Sync Project with Gradle Files```
  ![Sync Gradle](https://i.stack.imgur.com/Thqbc.png)
 
### 4. Installing Kanaloa Android Apps to phone
 
### 5. Using Kanaloa Android Apps
 
### 6. Sensors used in location app
