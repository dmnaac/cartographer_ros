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
template <typename Type>
Type getRosParam(ros::NodeHandle &nodehandle, const std::string &param_name, const Type &default_value)
{
  Type param_value;
  nodehandle.param<Type>(param_name, param_value, default_value);
  return param_value;
}

namespace cartographer_ros
{
  namespace
  {

    struct pbstreamIndex
    {
      geometry_msgs::Pose initialpose;
      std::string file_name;
      int trajectory_id;
    };

    cartographer_ros::Node *node_handle;
    cartographer_ros::TrajectoryOptions *trajectory_options_handle;
    bool isLocalizationMode = false;
    std::vector<pbstreamIndex> pbstreams;
    std::string last_filename;
    bool firstUpdate = true;

    bool isEqual(double a, double b, double epsilon = 1e-9)
    {
      return std::fabs(a - b) < epsilon;
    }

    void SetInitialPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
    {
      if (isLocalizationMode)
      {
        // Close current trajectories
        node_handle->FinishAllTrajectories();

        ros::NodeHandle nodehandle("~");
        last_filename = FLAGS_load_state_filename;
        const geometry_msgs::Pose init = msg->pose.pose;
        std::string current_floor_name;
        bool foundMatched = false;
        bool hasTrajectory = false;
        int current_trajectory_id = 0;
        std::vector<int> trajectory_ids;

        for (const auto &elem : pbstreams)
        {
          if (isEqual(elem.initialpose.position.x, init.position.x) && isEqual(elem.initialpose.position.y, init.position.y))
          {
            current_floor_name = elem.file_name;
            if (!elem.trajectory_id < 0)
            {
              hasTrajectory = true;
              current_trajectory_id = elem.trajectory_id;
            }
            if (!last_filename == getRosParam<std::string>(nodehandle, current_floor_name, ""))
            {
              foundMatched = true;
              ROS_INFO("Found matched pbstream.");
            }
          }
        }

        if (foundMatched || cnt == 0)
        {
          for (const auto &entry : map_builder_bridge_.GetTrajectoryStates())
          {
            trajectory_ids.push_back(entry.first);
          }

          for (const auto &elem : pbstreams)
          {
            if (last_filename == getRosParam<std::string>(nodehandle, elem.file_name, ""))
            {
              elem.trajectory_id = trajectory_ids.back();
            }
          }
        }

        if (foundMatched)
        {
          if (hasTrajectory)
          {
            ::cartographer::mapping::proto::InitialTrajectoryPose initial_trajectory_pose;
            initial_trajectory_pose.set_to_trajectory_id(current_trajectory_id);
            *initial_trajectory_pose.mutable_relative_pose() = cartographer::transform::ToProto(cartographer_ros::ToRigid3d(msg->pose.pose));
            initial_trajectory_pose.set_timestamp(cartographer::common::ToUniversal(::cartographer_ros::FromRos(ros::Time(0))));
            *trajectory_options_handle->trajectory_builder_options.mutable_initial_trajectory_pose() = initial_trajectory_pose;
            int new_active_trajectory_id = AddTrajectory(*trajectory_options_handle);
            isLocalizationMode = true;
          }
          else
          {
            *trajectory_options_handle->trajectory_builder_options.mutable_initial_trajectory_pose()->mutable_relative_pose() = cartographer::transform::ToProto(cartographer_ros::ToRigid3d(msg->pose.pose));
            FLAGS_load_state_filename = getRosParam<std::string>(nodehandle, current_floor_name, "");
            node_handle->LoadState(FLAGS_load_state_filename, FLAGS_load_frozen_state);
            node_handle->StartTrajectoryWithDefaultTopics(*trajectory_options_handle);
            isLocalizationMode = true;
          }
        }
        else
        {
          *trajectory_options_handle->trajectory_builder_options.mutable_initial_trajectory_pose()->mutable_relative_pose() = cartographer::transform::ToProto(cartographer_ros::ToRigid3d(msg->pose.pose));
          node_handle->StartTrajectoryWithDefaultTopics(*trajectory_options_handle);
          isLocalizationMode = true;
        }
      }
    }

    void Run()
    {
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

      if (!FLAGS_load_state_filename.empty())
      {
        node.LoadState(FLAGS_load_state_filename, FLAGS_load_frozen_state);
        localization_mode_flag = true;
        int num_floors = getRosParam<int>(nodehandle, "/number_of_floors", 1);
        std::string floor;
        geometry_msgs::Pose tempPose;
        std::string file_name;
        for (int i = 0; i < num_floors; i++)
        {
          floor = "/FLOOR_" + std::to_string(i + 1);
          tempPose.position.x = getRosParam<double>(nodehandle, floor + "_init_pos_x", 0.0);
          tempPose.position.y = getRosParam<double>(nodehandle, floor + "_init_pos_y", 0.0);
          tempPose.position.z = 0.0;
          tempPose.orientation.x = 0.0;
          tempPose.orientation.y = 0.0;
          tempPose.orientation.z = 0.0;
          tempPose.orientation.w = 1.0;
          file_name = floor + "_pbstream";
          pbstreams.push_back({tempPose, file_name, -1});
        }
      }

      if (FLAGS_start_trajectory_with_default_topics)
      {
        node.StartTrajectoryWithDefaultTopics(trajectory_options);
      }

      ros::Subscriber initialpose_sub = node.node_handle()->subscribe("/initialpose", 1, SetInitialPose);

      ::ros::spin();

      node.FinishAllTrajectories();
      node.RunFinalOptimization();

      if (!FLAGS_save_state_filename.empty())
      {
        node.SerializeState(FLAGS_save_state_filename,
                            true /* include_unfinished_submaps */);
      }
    }

  } // namespace
} // namespace cartographer_ros

int main(int argc, char **argv)
{
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