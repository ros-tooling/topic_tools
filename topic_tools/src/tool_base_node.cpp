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
}

void ToolBaseNode::make_subscribe_unsubscribe_decisions()
{
  if (auto source_info = try_discover_source()) {
    // always relay same topic type and QoS profile as the first available source
    if (!topic_type_ || !qos_profile_ || *topic_type_ != source_info->first ||
      *qos_profile_ != source_info->second || !pub_)
    {
      topic_type_ = source_info->first;
      qos_profile_ = source_info->second;
      std::scoped_lock lock(pub_mutex_);
      pub_ = this->create_generic_publisher(output_topic_, *topic_type_, *qos_profile_);
    }
    // at this point it is certain that our publisher exists

    // If lazy, only subscribe to input_topic if there is at least 1 subscriber on the output_topic
    if (!lazy_ ||
      pub_->get_subscription_count() + pub_->get_intra_process_subscription_count() > 0)
    {
      // subscription exists already but needs changing if input_topic_ changes
      if (sub_ &&
        sub_->get_topic_name() != get_node_topics_interface()->resolve_topic_name(input_topic_))
      {
        sub_.reset();
      }
      // subscription needs creating if it doesn't exist
      if (!sub_) {
        sub_ = this->create_generic_subscription(
          input_topic_, *topic_type_, *qos_profile_,
          std::bind(&ToolBaseNode::process_message, this, std::placeholders::_1));
      }
    } else {
      // Lazy and no subscriber doesn't need to subscribe
      sub_.reset();
    }
  } else {
    // we don't have any source to republish, so we don't need a publisher
    // also, if the source topic type changes while it's offline this
    // prevents a crash due to mismatched topic types
    std::scoped_lock lock(pub_mutex_);
    pub_.reset();
  }
}

std::optional<std::pair<std::string, rclcpp::QoS>> ToolBaseNode::try_discover_source()
{
  // borrowed this from domain bridge
  // (https://github.com/ros2/domain_bridge/blob/main/src/domain_bridge/wait_for_graph_events.hpp)
  // Query QoS info for publishers
  std::vector<rclcpp::TopicEndpointInfo> endpoint_info_vec =
    this->get_publishers_info_by_topic(input_topic_);
  std::size_t num_endpoints = endpoint_info_vec.size();

  // If there are no publishers, return an empty optional
  if (num_endpoints < 1u) {
    return {};
  }

  // Initialize QoS
  rclcpp::QoS qos{10};
  // Default reliability and durability to value of first endpoint
  qos.reliability(endpoint_info_vec[0].qos_profile().reliability());
  qos.durability(endpoint_info_vec[0].qos_profile().durability());
  // Always use automatic liveliness
  qos.liveliness(rclcpp::LivelinessPolicy::Automatic);

  // Reliability and durability policies can cause trouble with enpoint matching
  // Count number of "reliable" publishers and number of "transient local" publishers
  std::size_t reliable_count = 0u;
  std::size_t transient_local_count = 0u;
  // For duration-based policies, note the largest value to ensure matching all publishers
  rclcpp::Duration max_deadline(0, 0u);
  rclcpp::Duration max_lifespan(0, 0u);
  for (const auto & info : endpoint_info_vec) {
    const auto & profile = info.qos_profile();
    if (profile.reliability() == rclcpp::ReliabilityPolicy::Reliable) {
      reliable_count++;
    }
    if (profile.durability() == rclcpp::DurabilityPolicy::TransientLocal) {
      transient_local_count++;
    }
    if (profile.deadline() > max_deadline) {
      max_deadline = profile.deadline();
    }
    if (profile.lifespan() > max_lifespan) {
      max_lifespan = profile.lifespan();
    }
  }

  // If not all publishers have a "reliable" policy, then use a "best effort" policy
  // and print a warning
  if (reliable_count > 0u && reliable_count != num_endpoints) {
    qos.best_effort();
    RCLCPP_WARN(
      this->get_logger(), "Some, but not all, publishers on topic %s "
      "offer 'reliable' reliability. Falling back to 'best effort' reliability in order"
      "to connect to all publishers.", input_topic_.c_str());
  }

  // If not all publishers have a "transient local" policy, then use a "volatile" policy
  // and print a warning
  if (transient_local_count > 0u && transient_local_count != num_endpoints) {
    qos.durability_volatile();
    RCLCPP_WARN(
      this->get_logger(), "Some, but not all, publishers on topic %s "
      "offer 'transient local' durability. Falling back to 'volatile' durability in order"
      "to connect to all publishers.", input_topic_.c_str());
  }

  qos.deadline(max_deadline);
  qos.lifespan(max_lifespan);

  if (!endpoint_info_vec.empty()) {
    return std::make_pair(endpoint_info_vec[0].topic_type(), qos);
  } else {
    return {};
  }
}
}  // namespace topic_tools
