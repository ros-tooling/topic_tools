// Copyright 2023 Martin Llofriu
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

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "topic_tools/throttle_node.hpp"
#include "test_topic_tool.hpp"

using namespace std::chrono_literals;

TEST_F(TestTopicToolSingleSub, MessagesAreEffectivelyThrottledByMessages) {
  auto options = rclcpp::NodeOptions{};
  options.append_parameter_override("input_topic", get_target_input_topic());
  options.append_parameter_override("output_topic", get_target_output_topic());

  constexpr double messages_per_sec = 1;
  options.append_parameter_override("throttle_type", "messages");
  options.append_parameter_override("msgs_per_sec", messages_per_sec);
  options.append_parameter_override("window", 1.0f);
  options.append_parameter_override("use_wall_clock", true);
  auto target_node = std::make_shared<topic_tools::ThrottleNode>(options);

  // Need to sleep 1 sec or the node will throttle the initial message
  std::this_thread::sleep_for(std::chrono::seconds(1));

  std::function<void(const std_msgs::msg::String::SharedPtr)> validator =
    [](const std_msgs::msg::String::SharedPtr msg) {
      ASSERT_EQ(msg->data, "not dropped");
    };

  int expected_messages = 0;
  for (std::string msg_content :
    {"not dropped", "dropped", "dropped", "dropped"})
  {
    set_msg_validator(validator);
    publish_and_check(msg_content, target_node);
  }
  expected_messages += 1;

  std::this_thread::sleep_for(std::chrono::seconds(1));

  for (std::string msg_content :
    {"not dropped", "dropped", "dropped", "dropped"})
  {
    set_msg_validator(validator);
    publish_and_check(msg_content, target_node);
  }
  expected_messages += 1;

  ASSERT_EQ(get_received_msgs(), expected_messages);
}

TEST_F(TestTopicToolSingleSub, MessagesAreEffectivelyThrottledByBandwith) {
  auto options = rclcpp::NodeOptions{};
  options.append_parameter_override("input_topic", get_target_input_topic());
  options.append_parameter_override("output_topic", get_target_output_topic());

  // Allow 24 bytes per second
  constexpr int bytes_per_sec = 24;
  options.append_parameter_override("throttle_type", "bytes");
  options.append_parameter_override("bytes_per_sec", bytes_per_sec);
  options.append_parameter_override("window", 1.0f);
  options.append_parameter_override("use_wall_clock", true);
  auto target_node = std::make_shared<topic_tools::ThrottleNode>(options);

  std::function<void(const std_msgs::msg::String::SharedPtr)> validator =
    [](const std_msgs::msg::String::SharedPtr msg) {
      ASSERT_EQ(msg->data, "not dropped");
    };

  int expected_messages = 0;
  for (std::string msg_content :
    {"not dropped", "not dropped", "dropped", "dropped", "dropped"})
  {
    set_msg_validator(validator);
    publish_and_check(msg_content, target_node);
  }
  expected_messages += 2;

  std::this_thread::sleep_for(std::chrono::seconds(1));

  for (std::string msg_content :
    {"not dropped", "not dropped", "dropped", "dropped", "dropped"})
  {
    set_msg_validator(validator);
    publish_and_check(msg_content, target_node);
  }
  expected_messages += 2;

  ASSERT_EQ(get_received_msgs(), expected_messages);
}
