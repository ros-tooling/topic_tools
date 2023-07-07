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

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class TestTopicTool : public ::testing::Test
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

  void set_msg_validator(const std::function<void(std_msgs::msg::String::ConstSharedPtr)> & f)
  {
    msg_validator_ = f;
  }

  int get_received_msgs()
  {
    return received_msgs_;
  }

protected:
  void topic_callback(std_msgs::msg::String::ConstSharedPtr msg)
  {
    msg_validator_(msg);
    received_msgs_++;
  }

private:
  std::function<void(std_msgs::msg::String::ConstSharedPtr)> msg_validator_;
  int received_msgs_{0};
};

class TestTopicToolSingleSub : public TestTopicTool
{
public:
  void SetUp()
  {
    using std::placeholders::_1;
    const std::string test_name =
      ::testing::UnitTest::GetInstance()->current_test_info()->name();
    test_node_ = rclcpp::Node::make_shared(test_name);
    target_input_topic_ = "/" + test_name + "/input";
    target_output_topic_ = "/" + test_name + "/output";
    subscription_ = test_node_->create_subscription<std_msgs::msg::String>(
      target_output_topic_, 10, std::bind(&TestTopicToolSingleSub::topic_callback, this, _1));
    publisher_ = test_node_->create_publisher<std_msgs::msg::String>(target_input_topic_, 10);
  }

  void TearDown()
  {
    test_node_.reset();
    subscription_.reset();
    publisher_.reset();
  }

  void publish_and_check(
    std::string msg_content,
    std::shared_ptr<rclcpp::Node> target_node)
  {
    auto message = std_msgs::msg::String();
    message.data = msg_content;
    publisher_->publish(message);
    rclcpp::spin_some(target_node);
    rclcpp::spin_some(test_node_);
  }

  std::string get_target_input_topic()
  {
    return target_input_topic_;
  }

  std::string get_target_output_topic()
  {
    return target_output_topic_;
  }

private:
  std::shared_ptr<rclcpp::Node> test_node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  std::string target_input_topic_;
  std::string target_output_topic_;
};

class TestTopicToolMultiSub : public TestTopicTool
{
public:
  void SetUp()
  {
    using std::placeholders::_1;
    const std::string test_name =
      ::testing::UnitTest::GetInstance()->current_test_info()->name();
    test_node_ = rclcpp::Node::make_shared(test_name);
    target_input_topic_prefix_ = "/" + test_name + "/input";
    target_output_topic_ = "/" + test_name + "/output";
    subscription_ = test_node_->create_subscription<std_msgs::msg::String>(
      target_output_topic_, 10, std::bind(&TestTopicToolMultiSub::topic_callback, this, _1));
    std::vector<std::string> topic_names = get_target_input_topics();
    for (size_t i = 0; i < topic_names.size(); i++) {
      publishers_[i] = test_node_->create_publisher<std_msgs::msg::String>(
        topic_names[i], 10);
    }
  }

  void TearDown()
  {
    test_node_.reset();
    subscription_.reset();
    for (int i = 0; i < num_target_input_topics_; i++) {
      publishers_[i].reset();
    }
  }

  void publish_and_check(
    std::string msg_content,
    int publisher_index,
    std::shared_ptr<rclcpp::Node> target_node)
  {
    auto message = std_msgs::msg::String();
    message.data = msg_content;
    assert(publisher_index < num_target_input_topics_);
    publishers_[publisher_index]->publish(message);
    rclcpp::spin_some(target_node);
    rclcpp::spin_some(test_node_);
  }

  std::vector<std::string> get_target_input_topics()
  {
    std::vector<std::string> res;
    for (int i = 0; i < num_target_input_topics_; i++) {
      res.push_back(target_input_topic_prefix_ + std::to_string(i));
    }
    return res;
  }

  std::string get_target_output_topic()
  {
    return target_output_topic_;
  }

protected:
  std::shared_ptr<rclcpp::Node> test_node_;

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  static constexpr int num_target_input_topics_ = 3;
  std::array<rclcpp::Publisher<std_msgs::msg::String>::SharedPtr,
    num_target_input_topics_> publishers_;
  std::string target_input_topic_prefix_;
  std::string target_output_topic_;
};
