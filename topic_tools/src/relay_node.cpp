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

#include <memory>
#include <optional> // NOLINT : https://github.com/ament/ament_lint/pull/324
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "topic_tools/relay_node.hpp"

namespace topic_tools
{
RelayNode::RelayNode(const rclcpp::NodeOptions & options)
: ToolBaseNode("relay", options)
{
  input_topic_ = declare_parameter<std::string>("input_topic");
  output_topic_ = declare_parameter<std::string>("output_topic", input_topic_ + "_relay");
  lazy_ = declare_parameter<bool>("lazy", false);

  discovery_timer_ = this->create_wall_timer(
    discovery_period_,
    std::bind(&RelayNode::make_subscribe_unsubscribe_decisions, this));

  make_subscribe_unsubscribe_decisions();
}

void RelayNode::process_message(std::shared_ptr<rclcpp::SerializedMessage> msg)
{
  std::scoped_lock lock(pub_mutex_);
  if (pub_) {
    pub_->publish(*msg);
  }
}

}  // namespace topic_tools

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(topic_tools::RelayNode)
