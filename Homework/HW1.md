# Homework #1
## Objectives:
- Ability to use the basic ROS elements.
  - Using overlay workspaces
    - Building a customized version of the turtlesim package.
  - Topics: Future assignment
  - Parameters: Set the "map" size via parameter
  - Services: Use existing turtlesim services to spawn turtles 
  - Launch files: Instatiate everything in one call
- Understanding coordinate frames and coordinate transforms
  - Use TF2 to determine the location of an obstacle in various coordinates
## Problems:
### 1. Overlay Workspace
Either download or Fork the [turtlesim github source files](https://github.com/ros/ros_tutorials/tree/foxy-devel/turtlesim) and build a customized version of the files in your own overlay workspace with the fillowing a customizations:
1. Change the window title to "name"_turtlesim
2. Change the workspace to be [40,20] in turtle coordinates (i.e. the turtle1/pose topic in the lower-left corner is [0,0] and the turtle1/pose in the upper-right corner is [40,20])
### 2. Parameters
1. Modify the turtlesim code to use a parameter to determine the workspace size.
### 3. Services
Create a node that uses the existing turtlesim services to configure the window as follows:
1. draw a black 10x10 cross at the center of the map (to eventually represent NED).
2. spawn a turtle at [20,10,0]_enu named usv
3. spawn a turtle at [25,15,facing 20,10]_enu named obstacle
4. spawn a turtle at [5,5,pi/2]_enu named observer
### 4. Launch files
1. Generate a launch file that starts the turtlesim node with name "name"_turtlesim
   - set the parameters appropriately
   - run your configure node
2. If you complete the TF2 part in problem 5, generate a launch file to run all of the above and tht TF2 node
   - NOTE: this means you will generate 2 different launch files
### 5. Coordinate systems
1. Draw the map with the following coordinate systems ENU located in lower left corner, NED located in center of the map, usv, obstacle, observer placed appropriately and oriented according to Fossen "base" convention
2. Generate a TF tree where the observer has information on the obstacle
3. Display the location of the obstacle with respect to the usv.
