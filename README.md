# custom_mapping

This github is a collection of packages designed to work along side the turtlebot3. All the custom packages required to operate the custom_mapping are here

custom_mapping uses 2D lidar and the intel real sense on the turtlbot along with the gmapping slam provided within the turtlebot packages. 

From those tools the custom_mapping is able to determine the difference between the perimeter and the objects inside the perimeter in the environment through 'assocication by neighbours' of the pixels in the OG_map and by finding the area foot print of the objects, Similar objects with the same foot prints are deemed to be the object. This enables the objects deemed to be part of the perimeter to be coloured with one colour and the objects another. 

With the objects identified, once the turtlebot 'looks' at them, with the intel real sense and point cloud projection, the real world colour of the object can be stored. Only storing the colours of the objects and ingoring the remaining.  

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