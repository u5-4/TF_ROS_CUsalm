// Copyright (c) 2026 u5-4
// SPDX-License-Identifier: Apache-2.0

#include <exception>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "cuvslam_localization_adapter/adapter_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<
      cuvslam_localization_adapter::CuvslamLocalizationAdapter>();
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("cuvslam_localization_adapter"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
