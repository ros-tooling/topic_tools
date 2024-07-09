// Copyright 2024 Rufus Wong
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

#include "topic_tools/demux_node.hpp"

#include <memory>
#include <optional>  // NOLINT : https://github.com/ament/ament_lint/pull/324
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace topic_tools
{
DemuxNode::DemuxNode(const rclcpp::NodeOptions & options)
: ToolBaseNode("demux", options)
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  std::string initial_topic = declare_parameter("initial_topic", "");
  input_topic_ = declare_parameter("input_topic", "~/input");
  lazy_ = false;
  output_topics_ = declare_parameter<std::vector<std::string>>("output_topics");
  if (initial_topic.empty()) {
    initial_topic = output_topics_.front();
  }
  output_topic_ = initial_topic;

  discovery_timer_ = this->create_wall_timer(
    discovery_period_, std::bind(&DemuxNode::make_subscribe_unsubscribe_decisions, this));

  make_subscribe_unsubscribe_decisions();

  demux_add_srv_ = create_service<topic_tools_interfaces::srv::DemuxAdd>(
    "~/add", std::bind(&DemuxNode::on_demux_add, this, _1, _2));
  demux_delete_srv_ = create_service<topic_tools_interfaces::srv::DemuxDelete>(
    "~/delete", std::bind(&DemuxNode::on_demux_delete, this, _1, _2));
  demux_list_srv_ = create_service<topic_tools_interfaces::srv::DemuxList>(
    "~/list", std::bind(&DemuxNode::on_demux_list, this, _1, _2));
  demux_select_srv_ = create_service<topic_tools_interfaces::srv::DemuxSelect>(
    "~/select", std::bind(&DemuxNode::on_demux_select, this, _1, _2));
}

void DemuxNode::make_subscribe_unsubscribe_decisions()
{
  if (
    output_topic_ != NONE_TOPIC ||
    std::find(output_topics_.begin(), output_topics_.end(), output_topic_) !=
    output_topics_.end())
  {
    ToolBaseNode::make_subscribe_unsubscribe_decisions();
  }
}

void DemuxNode::process_message(std::shared_ptr<rclcpp::SerializedMessage> msg)
{
  std::scoped_lock lock(pub_mutex_);
  if (!pub_) {
    return;
  }
  pub_->publish(*msg);
}

void DemuxNode::on_demux_add(
  const std::shared_ptr<topic_tools_interfaces::srv::DemuxAdd::Request> request,
  std::shared_ptr<topic_tools_interfaces::srv::DemuxAdd::Response> response)
{
  RCLCPP_INFO(get_logger(), "trying to add %s to demux", request->topic.c_str());

  if (request->topic == NONE_TOPIC) {
    RCLCPP_WARN(
      get_logger(), "failed to add topic %s to demux, because it's reserved for special use",
      request->topic.c_str());
    response->success = false;
    return;
  }

  if (
    std::find(output_topics_.begin(), output_topics_.end(), request->topic) !=
    output_topics_.end())
  {
    RCLCPP_WARN(
      get_logger(), "tried to add a topic that demux was already listening to: [%s]",
      request->topic.c_str());
    response->success = false;
    return;
  }

  output_topics_.push_back(request->topic);

  RCLCPP_INFO(get_logger(), "added %s to demux", request->topic.c_str());
  response->success = true;
}

void DemuxNode::on_demux_delete(
  const std::shared_ptr<topic_tools_interfaces::srv::DemuxDelete::Request> request,
  std::shared_ptr<topic_tools_interfaces::srv::DemuxDelete::Response> response)
{
  RCLCPP_INFO(get_logger(), "trying to delete %s to demux", request->topic.c_str());

  auto it = std::find_if(
    output_topics_.begin(), output_topics_.end(), [this, &request](const std::string & topic) {
      return get_node_topics_interface()->resolve_topic_name(topic) ==
             get_node_topics_interface()->resolve_topic_name(request->topic);
    });
  if (it != output_topics_.end()) {
    if (
      get_node_topics_interface()->resolve_topic_name(output_topic_) ==
      get_node_topics_interface()->resolve_topic_name(request->topic))
    {
      RCLCPP_WARN(
        get_logger(), "tried to delete currently selected topic %s from demux",
        request->topic.c_str());
      response->success = false;
      return;
    }
    output_topics_.erase(it);
    RCLCPP_INFO(get_logger(), "deleted topic %s from demux", request->topic.c_str());
    response->success = true;
    return;
  }

  RCLCPP_WARN(
    get_logger(), "tried to delete non-subscribed topic %s from demux", request->topic.c_str());
  response->success = false;
}

void DemuxNode::on_demux_list(
  [[maybe_unused]] const std::shared_ptr<topic_tools_interfaces::srv::DemuxList::Request> request,
  std::shared_ptr<topic_tools_interfaces::srv::DemuxList::Response> response)
{
  response->topics = output_topics_;
}

void DemuxNode::on_demux_select(
  const std::shared_ptr<topic_tools_interfaces::srv::DemuxSelect::Request> request,
  std::shared_ptr<topic_tools_interfaces::srv::DemuxSelect::Response> response)
{
  auto it =
    std::find_if(
    output_topics_.begin(), output_topics_.end(), [this](const std::string & topic) {
      return get_node_topics_interface()->resolve_topic_name(topic) ==
             get_node_topics_interface()->resolve_topic_name(output_topic_);
    });
  if (it != output_topics_.end()) {
    response->prev_topic = output_topic_;
  } else {
    response->prev_topic = "";
  }

  if (request->topic == NONE_TOPIC) {
    RCLCPP_INFO(get_logger(), "demux selected to no input.");
    output_topic_ = NONE_TOPIC;
    response->success = true;
  } else {
    RCLCPP_INFO(get_logger(), "trying to switch demux to %s", request->topic.c_str());
    it = std::find_if(
      output_topics_.begin(), output_topics_.end(), [this, &request](const std::string & topic) {
        return get_node_topics_interface()->resolve_topic_name(topic) ==
               get_node_topics_interface()->resolve_topic_name(request->topic);
      });
    if (it != output_topics_.end()) {
      output_topic_ = request->topic;
      make_subscribe_unsubscribe_decisions();
      RCLCPP_INFO(get_logger(), "demux selected input: [%s]", request->topic.c_str());
      response->success = true;
    }
  }
}

}  // namespace topic_tools

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(topic_tools::DemuxNode)
