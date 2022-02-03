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

#ifndef TOPIC_TOOLS__MUX_NODE_HPP_
#define TOPIC_TOOLS__MUX_NODE_HPP_

#include <memory>
#include <optional>  // NOLINT : https://github.com/ament/ament_lint/pull/324
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "topic_tools/tool_base_node.hpp"
#include "topic_tools_interfaces/srv/mux_add.hpp"
#include "topic_tools_interfaces/srv/mux_delete.hpp"
#include "topic_tools_interfaces/srv/mux_list.hpp"
#include "topic_tools_interfaces/srv/mux_select.hpp"

namespace topic_tools
{
static const char NONE_TOPIC[] = "__none";

class MuxNode final : public ToolBaseNode
{
public:
  TOPIC_TOOLS_PUBLIC
  explicit MuxNode(const rclcpp::NodeOptions & options);

private:
  void process_message(std::shared_ptr<rclcpp::SerializedMessage> msg) override;
  void make_subscribe_unsubscribe_decisions() override;
  void on_mux_add(
    const std::shared_ptr<topic_tools_interfaces::srv::MuxAdd::Request> request,
    std::shared_ptr<topic_tools_interfaces::srv::MuxAdd::Response> response);
  void on_mux_delete(
    const std::shared_ptr<topic_tools_interfaces::srv::MuxDelete::Request> request,
    std::shared_ptr<topic_tools_interfaces::srv::MuxDelete::Response> response);
  void on_mux_list(
    const std::shared_ptr<topic_tools_interfaces::srv::MuxList::Request> request,
    std::shared_ptr<topic_tools_interfaces::srv::MuxList::Response> response);
  void on_mux_select(
    const std::shared_ptr<topic_tools_interfaces::srv::MuxSelect::Request> request,
    std::shared_ptr<topic_tools_interfaces::srv::MuxSelect::Response> response);

  std::vector<std::string> input_topics_;
  rclcpp::Service<topic_tools_interfaces::srv::MuxAdd>::SharedPtr mux_add_srv_;
  rclcpp::Service<topic_tools_interfaces::srv::MuxDelete>::SharedPtr mux_delete_srv_;
  rclcpp::Service<topic_tools_interfaces::srv::MuxList>::SharedPtr mux_list_srv_;
  rclcpp::Service<topic_tools_interfaces::srv::MuxSelect>::SharedPtr mux_select_srv_;
};
}  // namespace topic_tools

#endif  // TOPIC_TOOLS__MUX_NODE_HPP_
