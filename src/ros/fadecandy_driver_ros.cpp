/*
 * Copyright (c) 2021 Eurotec, Netherlands
 * All rights reserved.
 *
 * Author: Jad Haj Mustafa
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "./fadecandy_driver_ros.h"

#include <rclcpp/logging.hpp>

namespace fadecandy_driver
{
FadecandyDriverROS::FadecandyDriverROS(rclcpp::Node::SharedPtr node, double restart_patience)
  : node_(node), diagnostic_updater_(node), restart_patience_(restart_patience)
{
  diagnostic_updater_.add("Info", this, &FadecandyDriverROS::diagnosticsCallback);
  diagnostics_timer_ =
      node_->create_wall_timer(std::chrono::seconds(1), std::bind(&FadecandyDriverROS::diagnosticsTimerCallback, this));

  connect_timer_ = node_->create_wall_timer(std::chrono::milliseconds(static_cast<uint64_t>(restart_patience_ * 1000)),
                                            std::bind(&FadecandyDriverROS::connectTimerCallback, this));

  led_subscriber_ = node_->create_subscription<fadecandy_msgs::msg::LEDArray>(
      "set_leds", 1, std::bind(&FadecandyDriverROS::setLedsCallback, this, std::placeholders::_1));
}

void FadecandyDriverROS::run()
{
  setupConnection();
  rclcpp::spin(node_);
}

void FadecandyDriverROS::setupConnection()
{
  try
  {
    auto serial_number = driver_.connect();
    diagnostic_updater_.setHardwareID(serial_number);
    RCLCPP_INFO(node_->get_logger(), "Fadecandy device is connected.");
  }
  catch (const std::exception& e)
  {
    RCLCPP_WARN_ONCE(node_->get_logger(), "Failed to connect to device: %s; will retry every %f seconds", e.what(),
                     restart_patience_);
  }
}

void FadecandyDriverROS::setLedsCallback(const fadecandy_msgs::msg::LEDArray::SharedPtr led_array_msg)
{
  if (!driver_.isConnected())
  {
    return;
  }

  std::vector<std::vector<Color>> led_array_colors;
  for (const auto& strip : led_array_msg->strips)
  {
    std::vector<Color> led_strip_colors;
    for (const auto& color : strip.colors)
    {
      led_strip_colors.emplace_back(static_cast<int>(color.r * 255), static_cast<int>(color.g * 255),
                                    static_cast<int>(color.b * 255));
    }
    led_array_colors.push_back(led_strip_colors);
  }

  try
  {
    driver_.setColors(led_array_colors);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(node_->get_logger(), "Error occured: %s ", e.what());
  }
}

void FadecandyDriverROS::diagnosticsCallback(diagnostic_updater::DiagnosticStatusWrapper& diagnostic_status)
{
  if (driver_.isConnected())
  {
    diagnostic_status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Connected");
  }
  else
  {
    diagnostic_status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Disconnected");
  }
}

void FadecandyDriverROS::diagnosticsTimerCallback()
{
  diagnostic_updater_.force_update();
}

void FadecandyDriverROS::connectTimerCallback()
{
  if (driver_.isConnected())
  {
    return;
  }
  setupConnection();
}
}  // namespace fadecandy_driver