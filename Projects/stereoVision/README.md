# Stereo Vision

Created by: Brennan Yamamoto
Last update: 2018.12.17

In this work, we calibrate a stereo camera, and use it to generate a 3D point cloud from an image pair.

## General workflow

### Step 1: Collect calibration images

A set of sample calibration images are located in the `2018.11.15_calibrationImages` directory.  However, an image collection script `m20181114_collectStereoImgSync.m` is provided if re-calibration is needed.  

This script requires that ROS is running and is publishing the two image topics `/stereoLeft/usb_cam/image_raw/compressed` and `/stereoRight/usb_cam/image_raw/compressed`.

### Step 2: Calibrate camera

Navigate to the `APPS` tab of the Matlab integrated development environment, and click on "Stereo Camera Calirator".  Alternatively, you can type `stereoCameraCalibrator` into the command window to launch it directly.  Start a new camera calibration session, and import your left camera images, and right camera images, being careful that you don't have these two image sets reversed (this is affect your ability to generate a point cloud later down the line).

You also need to specify the size of each square in the checkerboard used in your calibration images.  This is 22mm for the included image set.  When you are done calibration, export your calibration parameters as a `stereoParams.mat` file.

### Step 3: Run the `stereoVision` script

This script collects the left and right streaming images from ROS (requiring the same topics as in Step 1), and generates a stereo anaglyph (allows the user to view the 3D image using red and blue 3D glasses), disparity map, and 3D point cloud.  


