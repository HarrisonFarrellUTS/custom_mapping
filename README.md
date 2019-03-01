# custom_mapping

This is a collection of all the custom packages required to operate the custom_mapping



## Installation

The folders must be saved within the 'catkin_ws/src' folder. 
Along with the 4 custom packages to have it operatation you must install the turtlebot3 packages.
Information about the turtlebot3 packages can be found on the ROS wiki or in the turtlebot maunal. 

## Usage 

To run the custom_mapping simply run 

```bash 
roslaunch colour_map_2d turtlebot3_slam_colour_mapping.launch
```
This will create all the required nodes, pulishers and subscribers. 