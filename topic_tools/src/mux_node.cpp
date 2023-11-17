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
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "topic_tools/mux_node.hpp"

namespace topic_tools
{
MuxNode::MuxNode(const rclcpp::NodeOptions & options)
: ToolBaseNode("mux", options)
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  input_topic_ = declare_parameter("initial_topic", "");
  output_topic_ = declare_parameter("output_topic", "~/selected");
  lazy_ = declare_parameter<bool>("lazy", false);
  input_topics_ = declare_parameter<std::vector<std::string>>("input_topics");
  if (input_topic_.empty()) {
    input_topic_ = input_topics_.front();
  }

  discovery_timer_ = this->create_wall_timer(
    discovery_period_,
    std::bind(&MuxNode::make_subscribe_unsubscribe_decisions, this));

  make_subscribe_unsubscribe_decisions();

  mux_add_srv_ = create_service<topic_tools_interfaces::srv::MuxAdd>(
    "~/add", std::bind(&MuxNode::on_mux_add, this, _1, _2));
  mux_delete_srv_ = create_service<topic_tools_interfaces::srv::MuxDelete>(
    "~/delete", std::bind(&MuxNode::on_mux_delete, this, _1, _2));
  mux_list_srv_ = create_service<topic_tools_interfaces::srv::MuxList>(
    "~/list", std::bind(&MuxNode::on_mux_list, this, _1, _2));
  mux_select_srv_ = create_service<topic_tools_interfaces::srv::MuxSelect>(
    "~/select", std::bind(&MuxNode::on_mux_select, this, _1, _2));
}

void MuxNode::make_subscribe_unsubscribe_decisions()
{
  if (input_topic_ != NONE_TOPIC ||
    std::find(input_topics_.begin(), input_topics_.end(), input_topic_) != input_topics_.end())
  {
    ToolBaseNode::make_subscribe_unsubscribe_decisions();
  }
}

void MuxNode::process_message(std::shared_ptr<rclcpp::SerializedMessage> msg)
{
  std::scoped_lock lock(pub_mutex_);
  if (!pub_) {
    return;
  }
  pub_->publish(*msg);
}

void MuxNode::on_mux_add(
  const std::shared_ptr<topic_tools_interfaces::srv::MuxAdd::Request> request,
  std::shared_ptr<topic_tools_interfaces::srv::MuxAdd::Response> response)
{
  RCLCPP_INFO(get_logger(), "trying to add %s to mux", request->topic.c_str());

  if (request->topic == NONE_TOPIC) {
    RCLCPP_WARN(
      get_logger(), "failed to add topic %s to mux, because it's reserved for special use",
      request->topic.c_str());
    response->success = false;
    return;
  }

  if (std::find(
      input_topics_.begin(), input_topics_.end(),
      request->topic) != input_topics_.end())
  {
    RCLCPP_WARN(
      get_logger(),
      "tried to add a topic that mux was already listening to: [%s]",
      request->topic.c_str());
    response->success = false;
    return;
  }

  input_topics_.push_back(request->topic);

  RCLCPP_INFO(get_logger(), "added %s to mux", request->topic.c_str());
  response->success = true;
}

void MuxNode::on_mux_delete(
  const std::shared_ptr<topic_tools_interfaces::srv::MuxDelete::Request> request,
  std::shared_ptr<topic_tools_interfaces::srv::MuxDelete::Response> response)
{
  RCLCPP_INFO(get_logger(), "trying to delete %s to mux", request->topic.c_str());

  auto it = std::find_if(
    input_topics_.begin(), input_topics_.end(),
    [this, &request](const std::string & topic) {
      return get_node_topics_interface()->resolve_topic_name(
        topic) == get_node_topics_interface()->resolve_topic_name(request->topic);
    }
  );
  if (it != input_topics_.end()) {
    if (get_node_topics_interface()->resolve_topic_name(input_topic_) ==
      get_node_topics_interface()->resolve_topic_name(request->topic))
    {
      RCLCPP_WARN(
        get_logger(), "tried to delete currently selected topic %s from mux",
        request->topic.c_str());
      response->success = false;
      return;
    }
    input_topics_.erase(it);
    RCLCPP_INFO(get_logger(), "deleted topic %s from mux", request->topic.c_str());
    response->success = true;
    return;
  }

  RCLCPP_WARN(
    get_logger(), "tried to delete non-subscribed topic %s from mux",
    request->topic.c_str());
  response->success = false;
}

void MuxNode::on_mux_list(
  [[maybe_unused]] const std::shared_ptr<topic_tools_interfaces::srv::MuxList::Request> request,
  std::shared_ptr<topic_tools_interfaces::srv::MuxList::Response> response)
{
  response->topics = input_topics_;
}

void MuxNode::on_mux_select(
  const std::shared_ptr<topic_tools_interfaces::srv::MuxSelect::Request> request,
  std::shared_ptr<topic_tools_interfaces::srv::MuxSelect::Response> response)
{
  auto it = std::find_if(
    input_topics_.begin(), input_topics_.end(),
    [this](const std::string & topic) {
      return get_node_topics_interface()->resolve_topic_name(
        topic) == get_node_topics_interface()->resolve_topic_name(input_topic_);
    }
  );
  if (it != input_topics_.end()) {
    response->prev_topic = input_topic_;
  } else {
    response->prev_topic = "";
  }

  if (request->topic == NONE_TOPIC) {
    RCLCPP_INFO(get_logger(), "mux selected to no input.");
    input_topic_ = NONE_TOPIC;
    response->success = true;
  } else {
    RCLCPP_INFO(get_logger(), "trying to switch mux to %s", request->topic.c_str());
    it = std::find_if(
      input_topics_.begin(), input_topics_.end(),
      [this, &request](const std::string & topic) {
        return get_node_topics_interface()->resolve_topic_name(
          topic) == get_node_topics_interface()->resolve_topic_name(request->topic);
      }
    );
    if (it != input_topics_.end()) {
      input_topic_ = request->topic;
      make_subscribe_unsubscribe_decisions();
      RCLCPP_INFO(get_logger(), "mux selected input: [%s]", request->topic.c_str());
      response->success = true;
    }
  }
}

}  // namespace topic_tools

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(topic_tools::MuxNode)
