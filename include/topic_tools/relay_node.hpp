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
#include <optional>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"

namespace topic_tools
{
class RelayNode : public rclcpp::Node
{
public:
  explicit RelayNode(const rclcpp::NodeOptions & options);

private:
  void republish_message(std::shared_ptr<rclcpp::SerializedMessage> msg);
  void make_subscribe_unsubscribe_decisions();

  /// Returns an optional pair <topic type, QoS profile> of the first found source publishing
  /// on `input_topic_` if at least one source is found
  std::optional<std::pair<std::string, rclcpp::QoS>> try_discover_source();

  std::chrono::duration<float> discovery_period_ = std::chrono::milliseconds{100};
  rclcpp::GenericSubscription::SharedPtr sub_;
  rclcpp::GenericPublisher::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr discovery_timer_;
  std::string input_topic_;
  std::string output_topic_;
  bool lazy_;
  std::optional<std::string> topic_type_;
  std::optional<rclcpp::QoS> qos_profile_;
};
}  // namespace topic_tools
