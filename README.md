# cartographer_ros
Modified cartographer_ros package for tmrobot.

In main branch, the node_main.cc file in the cartographer_ros package is modified to specify a different initial pose for the robot during localization compared to mapping. The initial pose is provided in a launch file which starts the cartographer_ros node.

In initialpose branch, the node_main.cc file is modified to subscribe the /initialpose topic.

The topic /tracked_pose(geometry_msgs/PoseStamped) is changed to have the geometry_msgs/PoseWithCovarianceStamped message type in the node.cc file. This topic is only published if the parameter publish_tracked_pose is set to true.

In the addPbstream branch, the node_main.cc file has been modified from the initialpose branch to enable the cartographer_ros node to switch pbstream files during localization. This allows the robot using Cartographer for localization to change pbstream files when traveling between different floors.
