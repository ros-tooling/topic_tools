// Copyright 2022 AIT Austrian Institute of Technology GmbH
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
#include <utility>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "topic_tools/delay_node.hpp"

namespace topic_tools
{
using namespace std::chrono_literals; // NOLINT

DelayNode::DelayNode(const rclcpp::NodeOptions & options)
: ToolBaseNode("delay", options), delay_(0ns)
{
  input_topic_ = declare_parameter<std::string>("input_topic");
  output_topic_ = declare_parameter<std::string>("output_topic", input_topic_ + "_delay");
  delay_ = rclcpp::Duration::from_seconds(declare_parameter("delay", 0.0));
  use_wall_clock_ = declare_parameter("use_wall_clock", false);

  discovery_timer_ = this->create_wall_timer(
    discovery_period_,
    std::bind(&DelayNode::make_subscribe_unsubscribe_decisions, this));
  make_subscribe_unsubscribe_decisions();
}

void DelayNode::process_message(std::shared_ptr<rclcpp::SerializedMessage> msg)
{
  auto clock = use_wall_clock_ ? std::make_shared<rclcpp::Clock>() : this->get_clock();
  auto slot = timers_.insert(timers_.end(), nullptr);
  *slot = rclcpp::create_timer(
    this, clock, delay_, [this, slot, msg]() {
      if (pub_) {
        // Due to the way subscriber discovery is implemented in ToolBaseNode, it's possible that
        // the publisher is deleted before invocation of the delayed callback, so the ptr needs to
        // be checked.
        // The mutex default callback group of this timer and the discovery timer ensures that the
        // publisher is not deleted between check and publish invocation.
        pub_->publish(*msg);
      }
      timers_.erase(slot);
    });
}

}  // namespace topic_tools

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(topic_tools::DelayNode)
