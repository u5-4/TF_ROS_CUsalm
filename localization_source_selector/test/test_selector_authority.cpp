// Copyright 2026 u5-4
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <chrono>
#include <cstddef>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <localization_adapter_interfaces/msg/localization_source_candidate.hpp>
#include <localization_adapter_interfaces/msg/selected_pose_candidate.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include "localization_source_selector/selector_node.hpp"

namespace localization_source_selector
{
namespace
{

using localization_adapter_interfaces::msg::LocalizationSourceCandidate;
using localization_adapter_interfaces::msg::SelectedPoseCandidate;
using namespace std::chrono_literals;

class SelectorAuthority : public ::testing::Test
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

rclcpp::NodeOptions TestOptions()
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {
        rclcpp::Parameter("mode", "cuvslam_primary"),
        rclcpp::Parameter(
          "contract_file",
          std::string(TEST_CONFIG_DIR) + "/cuvslam_primary.contract.yaml"),
      });
  return options;
}

LocalizationSourceCandidate Candidate(
  rclcpp::Node & source,
  const double x = 3.0)
{
  LocalizationSourceCandidate message;
  message.header.stamp = source.get_clock()->now();
  message.header.frame_id = "odom";
  message.semantic_child_frame = "base_link";
  message.pose.position.x = x;
  message.pose.orientation.w = 1.0;
  message.source_id = "cuvslam";
  message.source_contract_id = "d435i_fcu_cuvslam_shadow_20260723_v2";
  message.authorization = "source_pose_candidate_only";
  return message;
}

void SpinFor(
  rclcpp::executors::SingleThreadedExecutor & executor,
  const std::chrono::milliseconds duration)
{
  const auto deadline = std::chrono::steady_clock::now() + duration;
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(2ms);
  }
}

TEST_F(SelectorAuthority, OnlyTheLaunchSelectedSourceCanProduceSelectedOutput)
{
  auto selector = std::make_shared<LocalizationSourceSelector>(TestOptions());
  auto source = std::make_shared<rclcpp::Node>("cuvslam_localization_adapter");
  auto unselected = std::make_shared<rclcpp::Node>("mocap_localization_adapter");
  auto observer = std::make_shared<rclcpp::Node>("selected_pose_observer");
  const auto qos =
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  auto source_publisher = source->create_publisher<LocalizationSourceCandidate>(
    "/localization/candidates/cuvslam/base_pose", qos);
  auto unselected_publisher = unselected->create_publisher<LocalizationSourceCandidate>(
    "/localization/candidates/mocap/base_pose", qos);
  std::vector<SelectedPoseCandidate> outputs;
  auto subscription = observer->create_subscription<SelectedPoseCandidate>(
    "/localization/selected/pose", qos,
    [&outputs](const SelectedPoseCandidate::ConstSharedPtr message) {
      outputs.push_back(*message);
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(selector);
  executor.add_node(source);
  executor.add_node(unselected);
  executor.add_node(observer);

  // Exercise the graph-cache startup window before normal discovery settles.
  source_publisher->publish(Candidate(*source));
  SpinFor(executor, 100ms);
  const std::size_t after_early_candidate = outputs.size();

  auto wrong = Candidate(*unselected);
  wrong.source_id = "mocap";
  wrong.header.frame_id = "mocap_world";
  wrong.source_contract_id = "droneyee207_mocap_shadow_20260722_v2";
  unselected_publisher->publish(wrong);
  SpinFor(executor, 20ms);
  EXPECT_EQ(outputs.size(), after_early_candidate);

  const auto first = Candidate(*source, 3.0);
  source_publisher->publish(first);
  SpinFor(executor, 80ms);
  ASSERT_GT(outputs.size(), after_early_candidate);
  const auto & selected = outputs.back();
  EXPECT_EQ(selected.header.stamp.sec, first.header.stamp.sec);
  EXPECT_EQ(selected.header.stamp.nanosec, first.header.stamp.nanosec);
  EXPECT_EQ(selected.header.frame_id, "map");
  EXPECT_EQ(selected.semantic_child_frame, "base_link");
  EXPECT_EQ(selected.mode, "cuvslam_primary");
  EXPECT_EQ(selected.authorization, "selected_pose_candidate_only");
  EXPECT_FALSE(selected.localization_epoch_id.empty());
  EXPECT_NEAR(selected.pose.position.x, 0.0, 1.0e-9);
  (void)subscription;
}

TEST_F(SelectorAuthority, DuplicateInputPublishersLatchWithoutFallback)
{
  auto selector = std::make_shared<LocalizationSourceSelector>(TestOptions());
  auto source = std::make_shared<rclcpp::Node>("cuvslam_localization_adapter");
  auto intruder = std::make_shared<rclcpp::Node>("candidate_intruder");
  auto observer = std::make_shared<rclcpp::Node>("duplicate_input_observer");
  const auto qos =
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  auto expected_publisher = source->create_publisher<LocalizationSourceCandidate>(
    "/localization/candidates/cuvslam/base_pose", qos);
  auto duplicate_publisher = intruder->create_publisher<LocalizationSourceCandidate>(
    "/localization/candidates/cuvslam/base_pose", qos);
  std::size_t output_count = 0U;
  auto subscription = observer->create_subscription<SelectedPoseCandidate>(
    "/localization/selected/pose", qos,
    [&output_count](const SelectedPoseCandidate::ConstSharedPtr) {++output_count;});

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(selector);
  executor.add_node(source);
  executor.add_node(intruder);
  executor.add_node(observer);
  SpinFor(executor, 200ms);
  expected_publisher->publish(Candidate(*source));
  SpinFor(executor, 100ms);
  EXPECT_EQ(output_count, 0U);

  duplicate_publisher.reset();
  executor.remove_node(intruder);
  intruder.reset();
  SpinFor(executor, 200ms);
  for (int index = 0; index < 12; ++index) {
    expected_publisher->publish(Candidate(*source, 0.01 * index));
    SpinFor(executor, 10ms);
  }
  EXPECT_EQ(output_count, 0U);
  (void)subscription;
}

TEST_F(SelectorAuthority, DuplicateSelectedOutputPublisherStopsSelectorOutput)
{
  auto selector = std::make_shared<LocalizationSourceSelector>(TestOptions());
  auto source = std::make_shared<rclcpp::Node>("cuvslam_localization_adapter");
  auto intruder = std::make_shared<rclcpp::Node>("selected_output_intruder");
  auto observer = std::make_shared<rclcpp::Node>("duplicate_output_observer");
  const auto qos =
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  auto source_publisher = source->create_publisher<LocalizationSourceCandidate>(
    "/localization/candidates/cuvslam/base_pose", qos);
  auto duplicate_output = intruder->create_publisher<SelectedPoseCandidate>(
    "/localization/selected/pose", qos);
  std::size_t output_count = 0U;
  auto subscription = observer->create_subscription<SelectedPoseCandidate>(
    "/localization/selected/pose", qos,
    [&output_count](const SelectedPoseCandidate::ConstSharedPtr) {++output_count;});

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(selector);
  executor.add_node(source);
  executor.add_node(intruder);
  executor.add_node(observer);
  SpinFor(executor, 200ms);
  source_publisher->publish(Candidate(*source));
  SpinFor(executor, 100ms);
  EXPECT_EQ(output_count, 0U);

  duplicate_output.reset();
  executor.remove_node(intruder);
  intruder.reset();
  SpinFor(executor, 200ms);
  for (int index = 0; index < 12; ++index) {
    source_publisher->publish(Candidate(*source, 0.01 * index));
    SpinFor(executor, 10ms);
  }
  EXPECT_EQ(output_count, 0U);
  (void)subscription;
}

}  // namespace
}  // namespace localization_source_selector
