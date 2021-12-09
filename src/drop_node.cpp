// Copyright 2021 Daisuke Nishimatsu
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
#include <optional> // NOLINT : https://github.com/ament/ament_lint/pull/324
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "topic_tools/drop_node.hpp"

namespace topic_tools
{
DropNode::DropNode(const rclcpp::NodeOptions & options)
: ToolBaseNode("drop", options)
{
  x_ = declare_parameter<int>("X");
  y_ = declare_parameter<int>("Y");
}

void DropNode::process_message(std::shared_ptr<rclcpp::SerializedMessage> msg)
{
  if (count_ >= x_) {
    pub_->publish(*msg);
  }
  ++count_;
  if (count_ >= y_) {
    count_ = 0;
  }
}

}  // namespace topic_tools

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(topic_tools::DropNode)
