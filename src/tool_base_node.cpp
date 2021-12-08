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
#include "topic_tools/tool_base_node.hpp"

namespace topic_tools
{
ToolBaseNode::ToolBaseNode(const std::string & node_name, const rclcpp::NodeOptions & options)
: rclcpp::Node(node_name, options)
{
  input_topic_ = declare_parameter<std::string>("input_topic");
  output_topic_ = declare_parameter<std::string>("output_topic", input_topic_ + "_" + node_name);
  lazy_ = declare_parameter<bool>("lazy", false);

  discovery_timer_ = this->create_wall_timer(
    discovery_period_,
    std::bind(&ToolBaseNode::make_subscribe_unsubscribe_decisions, this));

  make_subscribe_unsubscribe_decisions();
}

void ToolBaseNode::make_subscribe_unsubscribe_decisions()
{
  if (auto source_info = try_discover_source()) {
    // always relay same topic type and QoS profile as the first available source
    if (*topic_type_ != source_info->first || *qos_profile_ != source_info->second) {
      topic_type_ = source_info->first;
      qos_profile_ = source_info->second;
      pub_ = this->create_generic_publisher(output_topic_, *topic_type_, *qos_profile_);
    }

    // at this point it is certain that our publisher exists
    if (!lazy_ || pub_->get_subscription_count() > 0) {
      if (!sub_) {
        sub_ = this->create_generic_subscription(
          input_topic_, *topic_type_, *qos_profile_,
          std::bind(&ToolBaseNode::process_message, this, std::placeholders::_1));
      }
    } else {
      sub_.reset();
    }
  } else {
    // we don't have any source to republish, so we don't need a publisher
    // also, if the source topic type changes while it's offline this
    // prevents a crash due to mismatched topic types
    pub_.reset();
  }
}

std::optional<std::pair<std::string, rclcpp::QoS>> ToolBaseNode::try_discover_source()
{
  auto publishers_info = this->get_publishers_info_by_topic(input_topic_);
  std::optional<rclcpp::QoS> qos_profile;
  if (!publishers_info.empty()) {
    return std::make_pair(publishers_info[0].topic_type(), publishers_info[0].qos_profile());
  } else {
    return {};
  }
}
}  // namespace topic_tools
