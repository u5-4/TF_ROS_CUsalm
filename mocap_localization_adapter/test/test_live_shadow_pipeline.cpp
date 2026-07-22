// Copyright 2026 u5-4
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <chrono>
#include <cstddef>
#include <memory>
#include <optional>
#include <thread>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <localization_adapter_interfaces/msg/shadow_pose_candidate.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mocap_localization_adapter/adapter_node.hpp"

namespace mocap_localization_adapter
{
namespace
{

using localization_adapter_interfaces::msg::ShadowPoseCandidate;
using namespace std::chrono_literals;

class LiveShadowPipeline : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      int argc = 0;
      char ** argv = nullptr;
      rclcpp::init(argc, argv);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

TEST_F(LiveShadowPipeline, PublishesOnlyTypedCandidateAfterHealthWarmup)
{
  auto adapter = std::make_shared<MocapLocalizationAdapter>();
  auto source = std::make_shared<rclcpp::Node>("vrpn_client_node");
  auto observer = std::make_shared<rclcpp::Node>("shadow_candidate_observer");
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  auto pose_publisher = source->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/droneyee207/pose", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile());
  std::optional<ShadowPoseCandidate> candidate;
  auto candidate_subscription = observer->create_subscription<ShadowPoseCandidate>(
    "/localization/shadow/mocap/assumed_base_pose",
    qos,
    [&candidate](const ShadowPoseCandidate::ConstSharedPtr message) {candidate = *message;});

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(adapter);
  executor.add_node(source);
  executor.add_node(observer);
  for (std::size_t index = 0U; index < 30U; ++index) {
    executor.spin_some();
    std::this_thread::sleep_for(5ms);
  }

  const auto publish_pose = [&pose_publisher, &source, &executor]() {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = source->get_clock()->now();
      pose.header.frame_id = "world";
      pose.pose.position.x = 1.0;
      pose.pose.position.y = 2.0;
      pose.pose.position.z = 3.0;
      pose.pose.orientation.w = 1.0;
      pose_publisher->publish(pose);
      executor.spin_some();
    };
  const auto publish_batch = [&publish_pose]() {
      publish_pose();
      publish_pose();
      std::this_thread::sleep_for(14ms);
    };
  for (std::size_t index = 0U; index < 50U; ++index) {
    publish_batch();
  }
  EXPECT_FALSE(candidate.has_value());

  for (std::size_t index = 0U; index < 50U && !candidate.has_value(); ++index) {
    publish_batch();
  }
  for (std::size_t index = 0U; index < 30U && !candidate.has_value(); ++index) {
    publish_pose();
    std::this_thread::sleep_for(5ms);
  }
  for (std::size_t index = 0U; index < 30U && !candidate.has_value(); ++index) {
    executor.spin_some();
    std::this_thread::sleep_for(5ms);
  }

  ASSERT_TRUE(candidate.has_value());
  EXPECT_EQ(candidate->header.frame_id, "mocap_world");
  EXPECT_EQ(candidate->semantic_child_frame, "base_link");
  EXPECT_DOUBLE_EQ(candidate->pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(candidate->pose.position.y, 2.0);
  EXPECT_DOUBLE_EQ(candidate->pose.position.z, 3.0);
  EXPECT_EQ(candidate->contract_id, "droneyee207_mocap_shadow_20260722");
  EXPECT_EQ(candidate->authorization, "shadow_candidate_only");
  EXPECT_EQ(candidate->source_topic, "/droneyee207/pose");
  EXPECT_EQ(candidate->source_parent_frame, "world");
  EXPECT_EQ(candidate->source_child_frame, "mocap_rigid_body");
  EXPECT_EQ(
    candidate->source_world_axes,
    "right_handed_x_reference_y_left_z_up_local_lab");
  EXPECT_EQ(candidate->source_body_axes, "x_forward_y_left_z_up");
  EXPECT_FALSE(candidate->geographic_alignment_validated);
  EXPECT_EQ(candidate->world_alignment_status, "local_lab_identity_not_geographic");
  EXPECT_FALSE(candidate->world_alignment_approved);
  EXPECT_EQ(candidate->extrinsic_status, "assumed_coincident_not_measured");
  EXPECT_EQ(
    candidate->extrinsic_assumption_id,
    "four_markers_centered_at_fcu_imu_20260722");
  EXPECT_FALSE(candidate->extrinsic_approved);
  EXPECT_EQ(candidate->expected_source_revision, "vrpn_client_ros2@1b9731c");
  EXPECT_EQ(
    candidate->expected_timestamp_semantics,
    "jetson_ros_callback_time_use_server_time_false");
  EXPECT_FALSE(candidate->source_configuration_validated);
  EXPECT_FALSE(candidate->capture_time_validated);

  (void)candidate_subscription;
}

}  // namespace
}  // namespace mocap_localization_adapter
