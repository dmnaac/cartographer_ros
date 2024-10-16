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

// Template function for getting values of parameters in ROS server
template<typename Type>
Type getRosParam(ros::NodeHandle &nodehandle, const std::string &param_name, const Type &default_value){
  Type param_value;
  nodehandle.param<Type>(param_name, param_value, default_value);
  return param_value;
}

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

namespace cartographer_ros {
namespace {

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

  // Set specified initial pose
  auto *trajectory_options_handle = &(trajectory_options);
  ros::NodeHandle nodehandle("~");
  double translation_x = getRosParam<double>(nodehandle, "/initial_translation_x", 0.0);
  double translation_y = getRosParam<double>(nodehandle, "/initial_translation_y", 0.0);
  double translation_z = getRosParam<double>(nodehandle, "/initial_translation_z", 0.0);
  double rotation_x = getRosParam<double>(nodehandle, "/initial_rotation_x", 0.0);
  double rotation_y = getRosParam<double>(nodehandle, "/initial_rotation_y", 0.0);
  double rotation_z = getRosParam<double>(nodehandle, "/initial_rotation_z", 0.0);
  double rotation_w = getRosParam<double>(nodehandle, "/initial_rotation_w", 1.0);
  geometry_msgs::Pose init_pose;
  init_pose.position.x = translation_x;
  init_pose.position.y = translation_y;
  init_pose.position.z = translation_z;
  init_pose.orientation.x = rotation_x;
  init_pose.orientation.y = rotation_y;
  init_pose.orientation.z = rotation_z;
  init_pose.orientation.w = rotation_w;
  *trajectory_options_handle->trajectory_builder_options.mutable_initial_trajectory_pose()->mutable_relative_pose()
  =cartographer::transform::ToProto(cartographer_ros::ToRigid3d(init_pose));


  if (!FLAGS_load_state_filename.empty()) {
    node.LoadState(FLAGS_load_state_filename, FLAGS_load_frozen_state);
  }

  if (FLAGS_start_trajectory_with_default_topics) {
    node.StartTrajectoryWithDefaultTopics(trajectory_options);
  }

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
