# Robot Localization

I have included the two versions of my launch file that have worked with the two phone bagfiles I have gotten. Also, instructions on how to use them as well as various other information so someone can continue from where I left off.

### Prerequisites

Before you can use robot localization, you will need ubuntu and a recent version of ROS installed on your computer.

### Installing

To install open terminal and type:

```
sudo apt-get install ros-kinetic-robot-localization
```

## Running robot_localization

To launch robot_localization:
* use terminal to navigate to the folder the file is located
* type roslaunch [name_of_launchfile]

Using the files I have created that would be:
```
roslaunch SecondaryPhoneLocalization.launch OR roslaunch TrimblesPhoneLocalization.launch
```
__Important Note:__ If you need to restart you input sensors (ex. bagfile ends or realife sensors disconnect), make sure to restart the localization launchfile as well. Otherwise it gets confused at why it is all of a sudden changing location.


## Troubleshooting

A lot of things have gone wrong while trying to use robot_localization. Chances are there will be more problems in the future, due to trying new sensors, using different phones, or adding additional sources of position data. Here are a few good places to start looking if the output is not what you expected.

### Checks

* Check the input data from the different sensors to see if they look reasonable (using rostopic echo [sensor_topic])
* Check covariance of sensors and final outputs (using rostopic echo)
* Look for error messages in the terminal tab you opened the launchfile in
* Check the topic: /odometry/gps (the output of NavSat Transform)
* Use rqt_graph to check that all the topics are correctly connected
* Use rostopic echo topic to see if it is not outputing anything
* Less common but check that frames are attached correctly using (rosrun tf view_frames) to make a pdf


## Testing

### Easiest way is to use the ROS dashboard that was made for this class
		TAB1
			roscore
		TAB2
			cd /opt/ros/kinetic/lib/rosbridge_server/
			python rosbridge_websocket.py 
		TAB3
			rosrun rosapi rosapi_node
		BROWSER
			http://rip.eng.hawaii.edu/dashboard/
			ENTER localhost or ip_adress

### Alternative is to use rviz
		TAB1
			roscore
		TAB2
			rviz rviz
		IN RVIZ
			fixed frame: odom
			Add: Odometry
				Topic: /odometry/filtered
        
### Alternative2 is to echo topics and try to understand their data
		TAB1
			rostopic echo (/topic_name)
				topic will usually be /odometry/filtered or /gps/filtered
		

