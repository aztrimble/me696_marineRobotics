# me696_marineRobotics Style Guide

This document details the programming best practices, conventions, and style guides we will use in the course.

## 1. Directory and filename convention

### 1.1. Directory naming convention
All directories shall start with a capital letter. If the directory name has multiple words, capitalize the first letter of subsequent words; you may use underscores _ when necessary, but do so sparingly. No spaces in directory names. Example: `SampleDirectory`.

### 1.2. File naming convention
For files requiring manual version control, the filenames shall start with the date that file was originally created. This is in the format YYYYMMDD_name. name must start with a lowercase letter. If name has multiple words, capitalize the first letter of subsequent words; you may use underscores _ when necessary, but do so sparingly. No spaces in filenames. Example: 20180101_sampleFilename.extension. Examples of files that require manual version control include: code, notes, work in progress, etc. You will need to use your judgement when deciding which files need manual version control. Github automatically handles version control by nature; however, it makes sense to create manual versions of code for minor milestones.

For files that do not require manual version control, follow the same naming convention as above, omitting the date. Example: `sampleFilename.extension`.

### 1.3. Exceptions:

Matlab (in Linux) dislikes filenames that start with numbers. For this reason, Matlab script .m filenames should begin with the letter m. Other than this, the naming convention remains the same. Example: `m20180101_sampleMatlabFilename.m`

Arduino IDE (in Linux) dislikes filenames that start with numbers. It also prefers that .ino scrips sit inside a directory with the same name as the filename. For this reason, Arduino script .ino filenames should begin with the letter a. The script should sit inside a directory named identical to the filename. Other than this, the naming convention remains the same. Example: `.../a20180101_sampleArduinoFilename/a20180101_sampleArduinoFilename.ino`

## 2. ROS best practices
 - [Overview of ROS best practices](http://wiki.ros.org/BestPractices)
 - [Overview of ROS Enhancement Protocols](http://www.ros.org/reps/rep-0000.html)
 - [REP 103: Standard Units of Measure and Coordinate Conventions](http://www.ros.org/reps/rep-0103.html)
 - [REP 105: Coordinate Frames for Mobile Platforms](http://www.ros.org/reps/rep-0105.html)
 - [ROS Developers Guide](http://wiki.ros.org/DevelopersGuide)

## 3. Per-language style guide

### 3.1 C++
 - [ROS C++ style guide](http://wiki.ros.org/CppStyleGuide)
 - [Google C++ style guide](https://google.github.io/styleguide/cppguide.html)
 - [MIT OceanAI lab C++ coding guidelines](http://oceanai.mit.edu/ivpman/pmwiki/pmwiki.php?n=Lab.CPPStructure)
 - [MIT OceanAI lab C++ style guidelines](http://oceanai.mit.edu/ivpman/pmwiki/pmwiki.php?n=Lab.CPPStyle)

### 3.2 Java
 - [Google Java style guide](https://google.github.io/styleguide/javaguide.html)

### 3.3 Javascript
 - [ROS Javascript style guide](http://wiki.ros.org/JavaScriptStyleGuide)

### 3.4 Python
 - [ROS Python style guide](http://wiki.ros.org/PyStyleGuide)
 - [Google style guide](https://github.com/google/styleguide/blob/gh-pages/pyguide.md)
