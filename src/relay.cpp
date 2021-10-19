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


#include <rclcpp/rclcpp.hpp>

#include <rclcpp/generic_publisher.hpp>
#include <rclcpp/generic_subscription.hpp>

#include <memory>
#include <string>

namespace topic_tools
{

  class RelayNode : public rclcpp::Node
  {
  public:
    RelayNode(const std::string node_name, const rclcpp::NodeOptions &options);

  private:
    void republish_message(std::shared_ptr<rclcpp::SerializedMessage> msg);
    void make_subscribe_unsubscribe_decisions();
    void subscribe();
    void unsubscribe();
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

  RelayNode::RelayNode(
    const std::string node_name, const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) 
    : rclcpp::Node(node_name, options)
  {
    input_topic_ = declare_parameter<std::string>("input_topic");
    output_topic_ = declare_parameter<std::string>("output_topic", input_topic_ + "_relay");
    lazy_ = declare_parameter<bool>("lazy", false);

    discovery_timer_ = this->create_wall_timer(discovery_period_, 
                          std::bind(&RelayNode::make_subscribe_unsubscribe_decisions, this));

    make_subscribe_unsubscribe_decisions();
  }

  void RelayNode::make_subscribe_unsubscribe_decisions()
  {

    if (auto source_info = try_discover_source())
    {
      // always relay same topic type and QoS profile as the first available source
      if (*topic_type_ != source_info->first || *qos_profile_ != source_info->second)
      {
        std::cout << "topic type / qos profile changed\n";
        
        topic_type_ = source_info->first;
        qos_profile_ = source_info->second;
        pub_ = this->create_generic_publisher(output_topic_, *topic_type_, *qos_profile_);
      }

      // at this point it is certain that our publisher exists
      if (!lazy_ || pub_->get_subscription_count() > 0)
      {
        if (!sub_)
        {
          subscribe();
        }
      } 
      else
      {
        if (sub_) 
        {
          unsubscribe();
        }
      }
    }
    else
    {
      // we don't have any source to republish, so we don't need a publisher
      // also, if the source topic type changes while it's offline this
      // prevents a crash due to mismatched topic types
      pub_.reset();
    }
  }

  /* Since we don't know the topic type and QoS profile in advance we need to determine them from
  the input_topic_. */
  std::optional<std::pair<std::string, rclcpp::QoS>> RelayNode::try_discover_source()
  {
    auto publishers_info = this->get_publishers_info_by_topic(input_topic_);
    std::optional<rclcpp::QoS> qos_profile;
    if (!publishers_info.empty()) 
    {
      return std::make_pair(publishers_info[0].topic_type(), publishers_info[0].qos_profile());
    }
    else
    {
      return {};
    }
  }

  void RelayNode::subscribe()
  {
    sub_ = this->create_generic_subscription(
      input_topic_, *topic_type_, *qos_profile_, 
      std::bind(&RelayNode::republish_message, this, std::placeholders::_1));
  }

  void RelayNode::unsubscribe()
  {
    if (sub_) {
      sub_.reset();
    }
  }

  void RelayNode::republish_message(std::shared_ptr<rclcpp::SerializedMessage> msg)
  {
    rclcpp::SerializedMessage msg_rcl = *msg.get();
    pub_->publish(msg_rcl);
  }

} // namespace topic_tools

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  std::shared_ptr<topic_tools::RelayNode> node;
  auto options = rclcpp::NodeOptions();

  if (argc >= 2 && strcmp(argv[1], "--ros-args"))
  {
    options.append_parameter_override("input_topic", argv[1]);
    if (argc >= 3 && strcmp(argv[2], "--ros-args"))
    {
      options.append_parameter_override("output_topic", argv[2]);
    }
  }
  node = std::make_shared<topic_tools::RelayNode>("Relay", options);
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
