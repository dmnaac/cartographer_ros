/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "absl/memory/memory.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer_ros/node.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/ros_log_sink.h"
#include "gflags/gflags.h"
#include "tf2_ros/transform_listener.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <utility>

DEFINE_bool(collect_metrics, false,
            "Activates the collection of runtime metrics. If activated, the "
            "metrics can be accessed via a ROS service.");
DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");
DEFINE_string(load_state_filename, "",
              "If non-empty, filename of a .pbstream file to load, containing "
              "a saved SLAM state.");
DEFINE_bool(load_frozen_state, true,
            "Load the saved state as frozen (non-optimized) trajectories.");
DEFINE_bool(
    start_trajectory_with_default_topics, true,
    "Enable to immediately start the first trajectory with default topics.");
DEFINE_string(
    save_state_filename, "",
    "If non-empty, serialize state and write it to disk before shutting down.");

// Template function for getting values of parameters in ROS server
template<typename Type>
Type getRosParam(ros::NodeHandle &nodehandle, const std::string &param_name, const Type &default_value){
  Type param_value;
  nodehandle.param<Type>(param_name, param_value, default_value);
  return param_value;
}

namespace cartographer_ros {
namespace {

cartographer_ros::Node* node_handle;
cartographer_ros::TrajectoryOptions* trajectory_options_handle;
bool localization_mode_flag = false;
std::vector<std::pair<geometry_msgs::Pose, std::string>> pbstreams;

void SetInitialPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
  if (localization_mode_flag) {
    //Close current trajectories
    node_handle->FinishAllTrajectories();

    const geometry_msgs::Pose init = msg->pose.pose;
    std::string floor_name;
    for (const auto& pair : pbstreams) {
        if (pair.first.position.x == init.position.x || pair.first.position.y == init.position.y)
        {
          floor_name = pair.second;
        }    
    }

    //Set trajectory builder options with new initial pose
    *trajectory_options_handle->trajectory_builder_options.mutable_initial_trajectory_pose()->mutable_relative_pose()
    =cartographer::transform::ToProto(cartographer_ros::ToRigid3d(msg->pose.pose));

    if (!FLAGS_load_state_filename.empty()) {
      ros::NodeHandle nodehandle("~");
      FLAGS_load_state_filename = getRosParam<std::string>(nodehandle, floor_name, "");
      node_handle->LoadState(FLAGS_load_state_filename, FLAGS_load_frozen_state);
      localization_mode_flag = true;
    }

    //Start a new trajectory with new initial pose
    if (FLAGS_start_trajectory_with_default_topics) {
      node_handle->StartTrajectoryWithDefaultTopics(*trajectory_options_handle);
    }
  }
}

void Run() {
  constexpr double kTfBufferCacheTimeInSeconds = 10.;
  tf2_ros::Buffer tf_buffer{::ros::Duration(kTfBufferCacheTimeInSeconds)};
  tf2_ros::TransformListener tf(tf_buffer);
  NodeOptions node_options;
  TrajectoryOptions trajectory_options;
  std::tie(node_options, trajectory_options) =
      LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);

  auto map_builder =
      cartographer::mapping::CreateMapBuilder(node_options.map_builder_options);
  Node node(node_options, std::move(map_builder), &tf_buffer,
            FLAGS_collect_metrics);
  
  trajectory_options_handle = &(trajectory_options);
  node_handle = &(node);
  ros::NodeHandle nodehandle("~");

  int num_floors = getRosParam<int>(nodehandle, "/number_of_floors", 1);
  for (int i = 0; i < num_floors; i++)
  {
    std::string floor = "FLOOR_" + std::to_string(i+1);
    std::pair<geometry_msgs::Pose, std::string> tempPair;
    tempPair.first.position.x = getRosParam<double>(nodehandle, floor + "_init_pos_x", 0.0);
    tempPair.first.position.y = getRosParam<double>(nodehandle, floor + "_init_pos_y", 0.0);
    tempPair.first.position.z = 0.0;
    tempPair.second = floor + "_pbstream";
    pbstreams.push_back(tempPair);
  }
  
  if (!FLAGS_load_state_filename.empty()) {
    node.LoadState(FLAGS_load_state_filename, FLAGS_load_frozen_state);
    localization_mode_flag = true;
  }

  if (FLAGS_start_trajectory_with_default_topics) {
    node.StartTrajectoryWithDefaultTopics(trajectory_options);
  }

  ros::Subscriber initialpose_sub = node.node_handle()->subscribe("/initialpose", 1, SetInitialPose);

  ::ros::spin();

  node.FinishAllTrajectories();
  node.RunFinalOptimization();

  if (!FLAGS_save_state_filename.empty()) {
    node.SerializeState(FLAGS_save_state_filename,
                        true /* include_unfinished_submaps */);
  }
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_configuration_directory.empty())
      << "-configuration_directory is missing.";
  CHECK(!FLAGS_configuration_basename.empty())
      << "-configuration_basename is missing.";

  ::ros::init(argc, argv, "cartographer_node");
  ::ros::start();

  cartographer_ros::ScopedRosLogSink ros_log_sink;
  cartographer_ros::Run();
  ::ros::shutdown();
}