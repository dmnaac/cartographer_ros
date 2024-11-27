# cartographer_ros

Modified cartographer_ros package for tmrobot.

This package now includes the following functionalities:

1. The cartographer_ros node subscribes to the /initialpose topic published by RViz. As a result, the robot using Cartographer for localization can now adjust its pose via RViz.

2. The topic /tracked_pose(geometry_msgs/PoseStamped) is changed to have the geometry_msgs/PoseWithCovarianceStamped message type in the node.cc file. This topic is only published if the parameter publish_tracked_pose is set to true.

3. A new service, switch_trajectory, has been added. This service is triggered when the robot moves beyond the current map and needs to load the corresponding pbstream file for the new location. It facilitates the robot's ability to navigate between different floors within a building.
