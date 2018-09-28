# Real-Time Image Recognition

The eventual goal of this work is to identify shapes streaming through a camera device in ROS.  The shapes to be identified are a square, circle, triangle, and cruciform (plus sign), as described in the RobotX [2016](https://www.robotx.org/images/files/2016-MRC-Tasks-2016-11-28.pdf) and [2018](https://www.robotx.org/images/RobotX-2018-Tasks_v2.0.pdf) rules.

## General workflow
To test this system initially, we will use the [Virtual Maritme RobotX Challenge simulation](https://bitbucket.org/osrf/vmrc).  While generating images from a simulation environment is far from perfect, using a simulation is a quick method of validating the interoperability of the required packages.  Eventually this functionality will be extended into the real world.  

To accomplish shape detection via image recognition, we will use a convolutional neural network in the Matlab deep learning toolbox.  

## 

