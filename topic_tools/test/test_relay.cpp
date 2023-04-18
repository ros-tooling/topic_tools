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

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "topic_tools/relay_node.hpp"

class TestRelay : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }
  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }
  void topic_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    ASSERT_EQ(msg->data, expected_msg);
    received_relays++;
  }

  std::string expected_msg;
  int received_relays{0};
};

TEST_F(TestRelay, MessagesAreEffectivelyRelayed) {
  using std::placeholders::_1;
  auto test_node = rclcpp::Node::make_shared("MessagesAreEffectivelyRelayed");
  auto subscription = test_node->create_subscription<std_msgs::msg::String>(
    "/relay_test/output", 10, std::bind(&TestRelay::topic_callback, this, _1));
  auto publisher = test_node->create_publisher<std_msgs::msg::String>("/relay_test/input", 10);
  rclcpp::spin_some(test_node);

  auto options = rclcpp::NodeOptions{};
  options.append_parameter_override("input_topic", "/relay_test/input");
  options.append_parameter_override("output_topic", "/relay_test/output");
  auto target_node = std::make_shared<topic_tools::RelayNode>(options);
  rclcpp::spin_some(target_node);

  for (std::string msg_content : {"hello", "again"}) {
    expected_msg = msg_content;

    auto message = std_msgs::msg::String();
    message.data = msg_content;
    publisher->publish(message);

    rclcpp::spin_some(test_node);
    rclcpp::spin_some(target_node);
    rclcpp::spin_some(test_node);
  }

  ASSERT_EQ(received_relays, 2);
}
