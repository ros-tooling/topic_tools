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

#include <memory>
#include <string>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "topic_tools/drop_node.hpp"
#include "test_topic_tool_single_sub.hpp"

TEST_F(TestTopicToolSingleSub, MessagesAreEffectivelyDropped) {
  auto options = rclcpp::NodeOptions{};
  options.append_parameter_override("input_topic", get_target_input_topic());
  options.append_parameter_override("output_topic", get_target_output_topic());
  // Drop 1 every two messages
  options.append_parameter_override("X", 1);
  options.append_parameter_override("Y", 2);
  auto target_node = std::make_shared<topic_tools::DropNode>(options);

  int published_msgs = 0;
  for (std::string msg_content :
    {"droped", "not dropped", "dropped", "not dropped", "droped"})
  {
    std::function<void(const std_msgs::msg::String::SharedPtr)> validator =
      [](const std_msgs::msg::String::SharedPtr msg) {
        ASSERT_EQ(msg->data, "not dropped");
      };
    set_msg_validator(validator);
    publish_and_check(msg_content, target_node);

    published_msgs++;
  }

  ASSERT_EQ(get_received_msgs(), published_msgs / 2);
}

TEST_F(TestTopicToolSingleSub, MessagesAreEffectivelyDroppedWithOddParameters) {
  auto options = rclcpp::NodeOptions{};
  options.append_parameter_override("input_topic", get_target_input_topic());
  options.append_parameter_override("output_topic", get_target_output_topic());
  // Drop 1 every two messages
  options.append_parameter_override("X", 3);
  options.append_parameter_override("Y", 5);
  auto target_node = std::make_shared<topic_tools::DropNode>(options);

  int published_msgs = 0;
  for (std::string msg_content :
    {"droped", "dropped", "dropped", "not dropped", "not dropped", "dropped"})
  {
    std::function<void(const std_msgs::msg::String::SharedPtr)> validator =
      [](const std_msgs::msg::String::SharedPtr msg) {
        ASSERT_EQ(msg->data, "not dropped");
      };
    set_msg_validator(validator);
    publish_and_check(msg_content, target_node);

    published_msgs++;
  }

  ASSERT_EQ(get_received_msgs(), (published_msgs / 5) * 2);
}
