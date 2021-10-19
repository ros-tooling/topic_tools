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

using namespace std::chrono_literals;

const auto LOOP_PERIOD = 100ms;

namespace topic_tools
{

  class RelayNode : public rclcpp::Node
  {
  public:
    RelayNode(
      const std::string node_name, 
      const rclcpp::NodeOptions &options);

  private:
    void incoming_message_callback(std::shared_ptr<rclcpp::SerializedMessage> msg);
    void timer_callback();
    void subscribe();
    void unsubscribe();
    std::optional<std::string> try_find_topic_type();

    rclcpp::GenericSubscription::SharedPtr sub_;
    rclcpp::GenericPublisher::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string input_topic;
    std::string output_topic;
    bool lazy;
    std::optional<std::string> topic_type;
  };

  RelayNode::RelayNode(
    const std::string node_name, const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) 
    : rclcpp::Node(node_name, options)
  {
    input_topic = declare_parameter<std::string>("input_topic");
    output_topic = declare_parameter<std::string>("output_topic", input_topic + "_relay");
    lazy = declare_parameter<bool>("lazy", false);

    auto ros_clock = rclcpp::Clock::make_shared();
    timer_ = rclcpp::create_timer(this, ros_clock, LOOP_PERIOD, 
                                  std::bind(&RelayNode::timer_callback, this));

    // so that we execute the loop the first time instantly
    timer_callback();
  }

  void RelayNode::timer_callback()
  {
    if (!topic_type)
    {
      if (topic_type = try_find_topic_type())
      {
        // we just found the topic type, so create the publisher and the subscriber
        pub_ = this->create_generic_publisher(output_topic, *topic_type, rclcpp::QoS(1));
        if (!lazy){
          subscribe();
        }
      }
    } 
    else if (lazy)
    {
      const size_t sub_count = pub_->get_subscription_count();
      if (sub_count > 0)
      {
        if (!sub_){
          subscribe();
        }
      } else
      {
        if (sub_) {
          unsubscribe();
        }
      }
    }
  }

  /*
Since we don't know the topic type in advance we need to determine it from
either the input_topic or the output_topic. If only one of them exists, we
use its type, but if both exist an arbitrary decision was made here to use
the type of input_topic.
*/
  std::optional<std::string> RelayNode::try_find_topic_type()
  {
    auto const topic_names_and_types = this->get_topic_names_and_types();
    auto topic_type_i = topic_names_and_types.find(input_topic);

    if (topic_type_i != topic_names_and_types.end())
    {
      return topic_type_i->second[0];
    }

    topic_type_i = topic_names_and_types.find(output_topic);

    if (topic_type_i != topic_names_and_types.end())
    {
      return topic_type_i->second[0];
    }

    return {};
  }

  void RelayNode::subscribe()
  {
    sub_ = this->create_generic_subscription(
      input_topic, *topic_type, rclcpp::QoS(1), 
      std::bind(&RelayNode::incoming_message_callback, this, std::placeholders::_1));
  }

  void RelayNode::unsubscribe()
  {
    if (sub_) {
      sub_.reset();
    }
  }

  void RelayNode::incoming_message_callback(std::shared_ptr<rclcpp::SerializedMessage> msg)
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
