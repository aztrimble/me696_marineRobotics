# Real-Time Image Recognition

The eventual goal of this work is to identify shapes streaming through a camera device in ROS.  The shapes to be identified are a square, circle, triangle, and cruciform (plus sign), as described in the RobotX [2016](https://www.robotx.org/images/files/2016-MRC-Tasks-2016-11-28.pdf) and [2018](https://www.robotx.org/images/RobotX-2018-Tasks_v2.0.pdf) rules.

## General workflow
To test this system initially, we will use the [Virtual Maritme RobotX Challenge (VMRC) simulation](https://bitbucket.org/osrf/vmrc).  While generating images from a simulation environment is far from perfect, using a simulation is a quick method of validating the interoperability of the required packages.  Eventually this functionality will be extended into the real world.  

To accomplish shape detection via image recognition, we will use a convolutional neural network in the Matlab deep learning toolbox.  

## Step 1: Collect and label test images
To generate test images, we will "drive" the WAM-V around in the VMRC simulation, and have Matlab subscribe to one of the camera topics.  In this implementation, we chose to subscibe to the `/front_left_camera/image_raw/compressed` topic, thought this may change depending on your simulation configuration.  

The script `m20180927_vmrcImageCollect.m` intializes a Matlab ROS node, and subcribes to the `/front_left_camera/image_raw/compressed` using the standard subscriber callback method.  The scipt allows the user to collect `n` number of images.  

## Step 2: 

