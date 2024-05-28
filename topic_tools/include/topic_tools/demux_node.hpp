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

#ifndef TOPIC_TOOLS__DEMUX_NODE_HPP_
#define TOPIC_TOOLS__DEMUX_NODE_HPP_

#include <memory>
#include <optional>  // NOLINT : https://github.com/ament/ament_lint/pull/324
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "topic_tools/tool_base_node.hpp"
#include "topic_tools_interfaces/srv/demux_add.hpp"
#include "topic_tools_interfaces/srv/demux_delete.hpp"
#include "topic_tools_interfaces/srv/demux_list.hpp"
#include "topic_tools_interfaces/srv/demux_select.hpp"

namespace topic_tools
{
static const char NONE_TOPIC[] = "__none";

class DemuxNode final : public ToolBaseNode
{
public:
  TOPIC_TOOLS_PUBLIC
  explicit DemuxNode(const rclcpp::NodeOptions & options);

private:
  void process_message(std::shared_ptr<rclcpp::SerializedMessage> msg) override;
  void make_subscribe_unsubscribe_decisions() override;
  void on_demux_add(
    const std::shared_ptr<topic_tools_interfaces::srv::DemuxAdd::Request> request,
    std::shared_ptr<topic_tools_interfaces::srv::DemuxAdd::Response> response);
  void on_demux_delete(
    const std::shared_ptr<topic_tools_interfaces::srv::DemuxDelete::Request> request,
    std::shared_ptr<topic_tools_interfaces::srv::DemuxDelete::Response> response);
  void on_demux_list(
    const std::shared_ptr<topic_tools_interfaces::srv::DemuxList::Request> request,
    std::shared_ptr<topic_tools_interfaces::srv::DemuxList::Response> response);
  void on_demux_select(
    const std::shared_ptr<topic_tools_interfaces::srv::DemuxSelect::Request> request,
    std::shared_ptr<topic_tools_interfaces::srv::DemuxSelect::Response> response);

  std::vector<std::string> output_topics_;
  rclcpp::Service<topic_tools_interfaces::srv::DemuxAdd>::SharedPtr demux_add_srv_;
  rclcpp::Service<topic_tools_interfaces::srv::DemuxDelete>::SharedPtr demux_delete_srv_;
  rclcpp::Service<topic_tools_interfaces::srv::DemuxList>::SharedPtr demux_list_srv_;
  rclcpp::Service<topic_tools_interfaces::srv::DemuxSelect>::SharedPtr demux_select_srv_;
};
}  // namespace topic_tools

#endif  // TOPIC_TOOLS__DEMUX_NODE_HPP_
