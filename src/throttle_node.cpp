// Copyright 2021 Mateusz Lichota
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

#include <deque>
#include <memory>
#include <optional> // NOLINT : https://github.com/ament/ament_lint/pull/324
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "topic_tools/throttle_node.hpp"

namespace topic_tools
{
ThrottleNode::ThrottleNode(const rclcpp::NodeOptions & options)
: ToolBaseNode("throttle", options)
{
  const std::string throttle_type_str = declare_parameter<std::string>("throttle_type");
  if (throttle_type_str == "messages") {
    throttle_type_ = ThrottleType::MESSAGES;
    msgs_per_sec_ = declare_parameter<double>("msgs_per_sec");
    period_ = rclcpp::Rate(msgs_per_sec_).period();
  } else if (throttle_type_str == "bytes") {
    throttle_type_ = ThrottleType::BYTES;
    bytes_per_sec_ = declare_parameter<int>("bytes_per_sec");
    window_ = declare_parameter<double>("window");
  } else {
    RCLCPP_ERROR(get_logger(), "Unknown throttle type");
    return;
  }
  use_wall_clock_ = declare_parameter("use_wall_clock", false);
  last_time_ = use_wall_clock_ ? rclcpp::Clock{}.now() : this->now();
}

void ThrottleNode::process_message(std::shared_ptr<rclcpp::SerializedMessage> msg)
{
  const auto & now = use_wall_clock_ ? rclcpp::Clock{}.now() : this->now();
  if (throttle_type_ == ThrottleType::MESSAGES) {
    if (last_time_ > now) {
      RCLCPP_WARN(
        get_logger(), "Detected jump back in time, resetting throttle period to now for.");
      last_time_ = now;
    }
    if ((now - last_time_).nanoseconds() >= period_.count()) {
      pub_->publish(*msg);
      last_time_ = now;
    }
  } else if (throttle_type_ == ThrottleType::BYTES) {
    while (!sent_deque_.empty() && sent_deque_.front().first < now.seconds() - window_) {
      sent_deque_.pop_front();
    }
    // sum up how many bytes are in the window
    const auto bytes =
      std::accumulate(
      sent_deque_.begin(), sent_deque_.end(), 0, [](const auto a, const auto & b) {
        return a + b.second;
      });
    if (bytes < bytes_per_sec_) {
      pub_->publish(*msg);
      sent_deque_.emplace_back(now.seconds(), msg->size());
    }
  }
}

}  // namespace topic_tools

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(topic_tools::ThrottleNode)
