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

#ifndef TOPIC_TOOLS__THROTTLE_NODE_HPP_
#define TOPIC_TOOLS__THROTTLE_NODE_HPP_

#include <deque>
#include <memory>
#include <optional>  // NOLINT : https://github.com/ament/ament_lint/pull/324
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "topic_tools/tool_base_node.hpp"
#include "topic_tools/visibility_control.h"

namespace topic_tools
{
class ThrottleNode final : public ToolBaseNode
{
  using Sent = std::pair<double, uint32_t>;

public:
  TOPIC_TOOLS_PUBLIC
  explicit ThrottleNode(const rclcpp::NodeOptions & options);

private:
  void process_message(std::shared_ptr<rclcpp::SerializedMessage> msg) override;

  enum class ThrottleType
  {
    MESSAGES,
    BYTES,
  } throttle_type_;
  double msgs_per_sec_;
  std::chrono::nanoseconds period_;
  int bytes_per_sec_;
  double window_;
  rclcpp::Time last_time_;
  bool use_wall_clock_;
  std::deque<Sent> sent_deque_;
};
}  // namespace topic_tools

#endif  // TOPIC_TOOLS__THROTTLE_NODE_HPP_
