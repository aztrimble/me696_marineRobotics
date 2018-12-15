# Real-Time Image Recognition

Created by: Brennan Yamamoto
Last update: 2018.12.15

In this work, we will identify shapes through a camera straming images through ROS.  We will do this using ROS and Matlab.  The shapes to be identified are a square, circle, triangle, and cruciform (plus sign), as described in the RobotX [2016](https://www.robotx.org/images/files/2016-MRC-Tasks-2016-11-28.pdf) and [2018](https://www.robotx.org/images/RobotX-2018-Tasks_v2.0.pdf) rules.

## Theory

### Machine learning background
At it's core, machine learning is most simply understood as nonlinear regression.  There are generally two classes of machine learning: classification, and regression learning.

In classification learning, you are tasked with identifying which "class" (or group), a particular feature belongs to.  Most classification algorithms are _binary classifiers_, i.e. they will identify either 0 (not belonging to a class) or 1 (belonging to a class); however, these binary classifiers can be turned into multi-classifiers through a variety of methods.  In classification learning, your "regression" algorithm attemps to characterize the line of separation that divides your two classes, based on a number of features/attributes.  This line of separation could be linear, or it could be some arbitrarily non-linear function; the challenge of machine learning is you generally do not know which function class your particular problem falls into.

In regression learning, you are tasked with predicting a range of values (as is the case with typical mathematical regression), i.e. instead of 0 or 1 as your output label (discrete), the output is continuous.  Instead of characterizing the line that separates two or more classes, you are directly characterizing the line that defines your function; however, similar to the classification problem, your true output function could be arbitrarily nonlinear, and you do not know which function class your problem falls into.

Some examples of classifcation problems are: identifying animals from images (dog vs. cat), predicting which team will win a football game, or identfying (biological) gender.  Some examples of regression problems are: predicting the price of a home, predicting a person's age, or preducting the correct medicine doseage for a hospital patient.

### The convolutional neural network

Artificial neural networks (ANNs) are a method of machine learning loosely based on the biological neural networks in animal brains.  In an ANN, a collection of nodes, called neurons, are constructed in layers, such that neurons in one layer can pass signals to neurons in the following layer.  The anatomy of a simple neuron is shown below:

In our problem, we are identifying whether or not a shape exists in an image, making it a classification problem.  In our case, we are distinguishing between three different possible shapes, making it a multi-classification problem.  In addition, this problem is made more complex still because it needs to identify whether or not multiple 

## General workflow
To test this system initially, we will use the [Virtual Maritme RobotX Challenge (VMRC) simulation](https://bitbucket.org/osrf/vmrc).  While generating images from a simulation environment is far from perfect, using a simulation is a quick method of validating the interoperability of the required packages.  Eventually this functionality will be extended into the real world.  

To accomplish shape detection via image recognition, we will use a convolutional neural network in the Matlab deep learning toolbox.  

## Step 1: Collect test images
To generate test images, we will "drive" the WAM-V around in the VMRC simulation, and have Matlab subscribe to one of the camera topics.  In this implementation, we chose to subscibe to the `/front_left_camera/image_raw/compressed` topic, thought this may change depending on your simulation configuration.  

The script `m20180927_vmrcImageCollect.m` intializes a Matlab ROS node, and subcribes to the `/front_left_camera/image_raw/compressed` using the standard subscriber callback method.  The scipt allows the user to collect `n` number of images at a set time interval, and writes these images to the current Matlab directory.  

## Step 2: Label test images
https://www.mathworks.com/help/vision/ug/train-object-detector-or-semantic-segmentation-network-from-ground-truth-data.html
https://www.mathworks.com/help/vision/ref/objectdetectortrainingdata.html

## Step 3: Create simple deep learning network for classification
https://www.mathworks.com/help/deeplearning/examples/create-simple-deep-learning-network-for-classification.html
https://www.mathworks.com/help/vision/examples/object-detection-using-deep-learning.html


## Step 4: Validate deep learning network against new streaming images from simulation

