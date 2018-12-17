# Real-Time Image Recognition

Created by: Brennan Yamamoto
Last update: 2018.12.16

In this work, we will identify shapes through a camera straming images through ROS.  We will do this using ROS and Matlab.  The shapes to be identified are a square, circle, triangle, and cruciform (plus sign), as described in the RobotX [2016](https://www.robotx.org/images/files/2016-MRC-Tasks-2016-11-28.pdf) and [2018](https://www.robotx.org/images/RobotX-2018-Tasks_v2.0.pdf) rules.

## Theory

### Machine learning background
At it's core, machine learning is most simply understood as nonlinear regression.  There are generally two classes of machine learning: classification, and regression learning.

In classification learning, you are tasked with identifying which "class" (or group), a particular feature belongs to.  Most classification algorithms are _binary classifiers_, i.e. they will identify either 0 (not belonging to a class) or 1 (belonging to a class); however, these binary classifiers can be turned into multi-classifiers through a variety of methods.  In classification learning, your "regression" algorithm attemps to characterize the line of separation that divides your two classes, based on a number of features/attributes.  This line of separation could be linear, or it could be some arbitrarily non-linear function; the challenge of machine learning is you generally do not know which function class your particular problem falls into.

In regression learning, you are tasked with predicting a range of values (as is the case with typical mathematical regression), i.e. instead of 0 or 1 as your output label (discrete), the output is continuous.  Instead of characterizing the line that separates two or more classes, you are directly characterizing the line that defines your function; however, similar to the classification problem, your true output function could be arbitrarily nonlinear, and you do not know which function class your problem falls into.

Some examples of classifcation problems are: identifying animals from images (dog vs. cat), predicting which team will win a football game, or identfying (biological) gender.  Some examples of regression problems are: predicting the price of a home, predicting a person's age, or preducting the correct medicine doseage for a hospital patient.

### The convolutional neural network

Artificial neural networks (ANNs) are a method of machine learning loosely based on the biological neural networks in animal brains.  In an ANN, a collection of nodes, called neurons, are constructed in layers, such that neurons in one layer can pass signals to neurons in the following layer.  The anatomy of a simple neuron is shown below:

![image](https://github.com/aztrimble/me696_marineRobotics/blob/master/Projects/realTimeImageRecognition/Images/neuron.jpg)

When a neuron receives a signal, it combines those signals into a weighting function, which is then multiplied by an activation function, φ, to determine its output. There are multiple candidates for the activation function, φ.  Common functions include the step function (perceptron), linear combination (no activation function), sigmoid function, tanh function, and rectified linear function.  A graphical representation of these activation functions are shown below:

![image](https://github.com/aztrimble/me696_marineRobotics/blob/master/Projects/realTimeImageRecognition/Images/activationFunctions.jpg)

By combining a network of these neurons togehter into a full ANN, comples functions can be modeled.  A standard forward-propagation model is shown below:

![image](https://github.com/aztrimble/me696_marineRobotics/blob/master/Projects/realTimeImageRecognition/Images/forwardPropagation.jpg)

The ANN in the above figure has two hidden laters--these are the layers in between the input and output layers in a given ANN.  The greater the number of hidden laters in an ANN, the greater it's intrinsic dimensionality, and therefore, the greater it's representational power.  For this reason, ANNs with many hidden layers are sometimes referred to as "deep learning" systems.  An ANN with a single neuron can express the boolearn functions AND, OR and NOT, but not XOR.  A feed-forward ANN with a single hidden layer can express any boolean function, and with a sufficiently large number of neurons in the hidden layer, it can approximate any bounded continuous function to arbitrary precision.  A feed-forward ANN with two hidden layers can approximate any function to aprbitrary accuracy.  

The word "convolutional" in the name refers to the connection scheme of the neurons.  In the forward propagation model shown in the above figure, all of the neurons in one layer are connected to all of the nodes in the following layer, and in the previous layer.  In a "convolutional" neural network, only the neurons in the near vicinity of other neurons are connected to each other.  The fully connected ANN is more mathematically simple, and provides the greatest theoretical representational power due to the total number of network connections; however, because the computation time increases exponentially with each neuron, the fully-connected ANN is computationally slow.  In problems involving visual data, there is a tendency for neurons to connect to neurons in their close vicinicy; this allows the convolutional neural network to train significantly more quickly, as there are less total connections between neurons.  Therefore, when normalizing for training time, convolutional neural networks can have significantly more hidden layers (and therefore, more "depth") than standard feed-forward neural networks when applied to problems involving vision.  

### The regions with convlutional neural network (R-CNN)

The regions with convolutional neural network, or R-CNN for short, is a machine learning method within Matlab.  In this method, the image is segmented into individual regions, then a convolutional neural network is run on those invidividual regions.  The output of this network results in multiple bounding boxes, ideally centered around features of interest (in our case, shapes).  The final step of the RCNN is to combine all similar bouding boxes in close proximity to each other into a single large bounding box with a single confidence.  When properly setup, this allows the RCNN to identify the positition and type of an arbitrary number of shapes in a single image, provided that two similar shapes are not in close proximity with each other (in which case, those two shapes will be identified as a single large shape.  This network type is idea for the RobotX competition, because the robot will need to distinguish between multiple shapes (and colors), which may be the same shape in different positions on the frame.  

## General workflow
To test this system initially, we will use the [Virtual Maritme RobotX Challenge (VMRC) simulation](https://bitbucket.org/osrf/vmrc).  While generating images from a simulation environment is far from perfect, using a simulation is a quick method of validating the interoperability of the required packages.  Eventually this functionality will be extended into the real world.  

### Step 1: Collect test images
To generate test images, we will "drive" the WAM-V around in the VMRC simulation, and have Matlab subscribe to one of the camera topics.  In this implementation, we chose to subscibe to the `/front_left_camera/image_raw/compressed` topic, thought this may change depending on your simulation configuration.  [For instructions on how to set up the VMRC simulation, visit this link](https://github.com/riplaboratory/Kanaloa/tree/master/Tutorials/SoftwareInstallation/RobotX-Simulation).

The script `m20181005_vmrcImageCollect.m` intializes a Matlab ROS node, and subcribes to the `/front_left_camera/image_raw/compressed` using the standard subscriber callback method.  The scipt allows the user to collect `n` number of images at a set time interval, and writes these images to the current Matlab directory.  

### Step 1b (optional): condition test images
At this point, you should have a database of images from the VMRC simulation saved to your working directory.  At the time of writing the VMRC simulation outputs images with a resolution of 800x800.  This resolution is quite strange since typical cameras utilize an aspect ratio fo 16:9, resulting in typical resolutions of 1920x1080, or 1280x720.  The images used to train the RCNN must all have the same resolution, therefore, it is important that you decide on a single resolution prior to importing all of your images to the RCNN.  It is a smart idea to utilize a resolution that matches the resolution of your camera, or is a factor/multiple of that resolution, so minimal pre-processing will need to be performed on each image frame when the RCNN is implemented in real time.  A higher resolution will result in an increased ability to resolve shapes at longer distances; however, extremely high resolutions will result in exponentially increased training time, _and_ increased execution time.  Therefore, our recommendation when selecting a scaled-down resolution that your camera internally supports, this will eliminate any pre-processing steps prior to running the network in real time.

For the sake of this example, the images will be scaled by a factor of 0.5 on each axis, which will take the output resolution of 800x800 to 400x400.  This can be performed using a simple scaling script on all images in your directory. 

### Step 2: Label test images
At this point, you should have a directory with a number of test and/or training images.  The next task is to "label" these images for training the RCNN in the next step.  Because the RCNN extracts _both_ the position of the feature in the image _and_ the type of feature, this also means that we need to label this information in training.  To do this, you should use the Matlab Image Labeler application.  Go to the `APPS` tab in the Matlab integrated development environment, and click on the Image Labeler application (alternatively, you can also type `imageLabeler` into the command window to launch the image labeler application directly).  

Before running the imageLabeler app, it is highly recommended that you put all of your training images in the same directory, and save your image labeling session in that same directory; __do not make subdirectories within your image directory__.  This way, the image labeler application is much less likely to run into path errors when adding new images to your image database.  Once inside the image labeler app, click on `Load > Add images from folder`, and select all of the images inside your training image directory for import.  The next thing you want to do is create noew ROI (region of interest) lables for each of your shapes.  In our case, our shapes are triangle, circle, and cruciform.  In each image, you will click and drag a bounding box on each shape corresponding to each ROI label.  A sample of this process is shown in the figure below: 

![image](https://github.com/aztrimble/me696_marineRobotics/blob/master/Projects/realTimeImageRecognition/Images/imageLabelerSample.JPG)

Continue this process for all of the images in your image datastore.  Once complete, save your image labeling session _and_ export your labels (either to your workspace, or as a `.mat` file).  Saving the image labeling session allows you to open your session at a later date if you add more training data to your directory, and your exported labels is the actual datastore imported into your RCNN for training.

### Step 3: Create and train RCNN network
The script `m20181005_trainRCNN.m` creates and trains your RCNN network.  First, ensure that your groundTruth variable path is correctly set in Matlab.  Depending on how you exported your groundTruth variable from your image labeling session, this line may need to be changed; the `trainingData` variable must ultimately point to your groundTruth variable in code.  

Second, also ensure that the `height` and `width` variables match the resolution of your imported images.  For this exercise, we will be using 400x400 images, but this is not a realistic resolution to use for real cameras (as discussed previously), so you will likely need to change this resolution to some of the more common 16:9 resolutions.  

Once your settings are correcly dialed in, run the script.  Provided that you have a Nvidia grahpics card on your computer, and the Matlab parallel computing toolbox in your Matlab installation, your training should execute much more quickly.  As more and more training epochs execute, you should see that your Mini-batch accuracy converges to 100%.  This can also give you an idea of how many epochs are necessary to achieve a reasonably accurate RCNN on a given training dataset. 

Once the training is complete, save the generated rcnn variable as a .mat file so that you can access it later.  The easiest way to do this is to right click the `rcnn` variable in the workspace, click `Save As...`, and save the file as `rcnn.mat`.  If you want to bring the rcnn back into your workspace in the future, simply double click the `rcnn.mat` file, and it will load into your workspace. 

### Step 4: Validate deep learning network against new streaming images from simulation
The script `m20181005_rosRcnnNode.m` runs alongside the VMRC simulation, and similar to the previous script, subscibes to the `/front_left_camera/image_raw/compressed` topic.  First, bring your RCNN into your workspace by double clicking your `rcnn.mat` file generated in the previous step (this may take a few seconds).  Once the VMRC simulation is launched, run the scipt; as you move the WAMV through the simulation environment, the script reads in the camera topic, and runs the straming image through the RCNN network.  The output from the network will publish to the topic `/rcnn_front_left_camera/image_raw/`, which you can then view in RVIZ, or any image viewing package in ROS.

If the network is sufficiently trained, as you approach the obstacle with shapes in the simulation, the script will apply a bounding box to each identified shape, and display the shape (`o` for circle, `+` for cruiciform, and `^` for triangle) and confidence (0 through 1).
