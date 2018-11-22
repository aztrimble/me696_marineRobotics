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
          + For the first command, it might be better to rename top directory another name (e.g ```mkdir -p ~/android_ros_workspace```), rather name the directory "android_core". The rename will just make it easier to track which folder you are in and won't have a path that looks like "~/android_core/src/android_core". Make sure you are consistant with the rest of the directory name with the rest of the commands. 
          + Be sure to run ```catkin_make``` before moving on to next steps
  
### 3. Importing Kanaloa Android App modules
  - Download/clone this repo to a folder on your computer
  - Open Android Studios
  - Import Module ```File -> New -> Import Module```
  ![Import module](https://i.stack.imgur.com/Nlpfo.png)
  - Find and Select the ```android_kanaloa_*``` module (folder) you want to import
  - Press the Sync Gradle project ```File -> Sync Project with Gradle Files```
  ![Sync Gradle](https://i.stack.imgur.com/Thqbc.png)
 
### 4. Build and run your app
You can 1) **Instant run module on phone** or 2) **Build all modules then install on phone**
  - 1)**Instant run module on phone**
      * Connect android phone to computer
      * Select the module you want to run from the drop down menu
      * Follow the Instructions in the Android [Build and run your app](https://developer.android.com/studio/run/) Tutorial 
  - 2)**Build all modules then install on phone**
      * select ```Build Bundle(s) / APK(s) > Build APK(s)```
      * then look for the apk in the build folder```project-name/module-name/build/outputs/apk/```
      * download apk to phone
      * on the phone, run apk to install app on phone
  
### 5. Using Kanaloa Android Apps
 
### 6. Sensors used in location app
  - GPS
      * [Location Manager](https://developer.android.com/reference/android/location/LocationManager) from [android.location](https://developer.android.com/reference/android/location/package-summary)
      * If onboard [GPS_PROVIDER](https://developer.android.com/reference/android/location/LocationManager#GPS_PROVIDER) module is avaliable use that information to update GPS position
      * otherwise, use the [NETWORK_PROVIDER](https://developer.android.com/reference/android/location/LocationManager#NETWORK_PROVIDER) to update the GPS position
  - IMU
      * World Frame
          + **Orientation:** [TYPE_ROTATION_VECTOR](https://developer.android.com/guide/topics/sensors/sensors_motion) is used. It combines gyroscope and magnetic field sensors to orient basted on true north.
          + **Angular Velocity:** [TYPE_GYROSCOPE](https://developer.android.com/guide/topics/sensors/sensors_motion) is used. It is calibrated to compensate for drift.
          + **Linear Acceleration:** [TYPE_LINEAR_ACCELERATION](https://developer.android.com/guide/topics/sensors/sensors_motion) is used. It gives readings that exclude gravity.
      * Device Frame
          + **Orientation:** [TYPE_GAME_ROTATION_VECTOR](https://developer.android.com/guide/topics/sensors/sensors_position) is used. This sensor doesn't use the geomagnetic field, Therefore the readings will be referneced to phone's position when app was launched.
          + **Angular Velocity:** [TYPE_GYROSCOPE](https://developer.android.com/guide/topics/sensors/sensors_motion) is used. It is calibrated to compensate for drift.
          + **Linear Acceleration:** [TYPE_LINEAR_ACCELERATION](https://developer.android.com/guide/topics/sensors/sensors_motion) is used. It gives readings that exclude gravity.
