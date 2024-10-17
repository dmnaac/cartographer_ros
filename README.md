# cartographer_ros
Modified cartographer_ros package for tmrobot.

In main branch, the node_main.cc file in the cartographer_ros package is modified to specify a different initial pose for the robot during localization compared to mapping. The initial pose is provided in a launch file which starts the cartographer_ros node.

In initialpose branch, the node_main.cc file is modified to subscribe the /initialpose topic.
