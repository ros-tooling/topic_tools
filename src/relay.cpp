#include <rclcpp/rclcpp.hpp>

#include <rclcpp/generic_publisher.hpp>
#include <rclcpp/generic_subscription.hpp>

#include <memory>
#include <string>

namespace topic_tools
{

class RelayNode : public rclcpp::Node {
  public:
    RelayNode();

  private:
    void incoming_message_callback(std::shared_ptr<rclcpp::SerializedMessage> msg);
    void subscribe();
    void unsubscribe();

    rclcpp::GenericSubscription::SharedPtr sub_;
    rclcpp::GenericPublisher::SharedPtr pub_;
    std::string input_topic;
    std::string output_topic;
    std::string topic_type;
};

RelayNode::RelayNode() : rclcpp::Node("Relay") {
  input_topic = declare_parameter<std::string>("input_topic");
  output_topic = declare_parameter<std::string>("output_topic", input_topic + "_relay");

  auto topic_names_and_types = this->get_topic_names_and_types();
  auto topic_type_i = topic_names_and_types.find(input_topic);

  if (topic_type_i == topic_names_and_types.end()) {
    std::cout << "error: input topic doesn't exist" << std::endl;
    return;
  }

  topic_type = topic_type_i->second[0];
  

  pub_ = this->create_generic_publisher(output_topic, topic_type, rclcpp::QoS(1));

  subscribe();
}

void RelayNode::subscribe() {
  sub_ = this->create_generic_subscription(input_topic, topic_type, rclcpp::QoS(1), std::bind(&RelayNode::incoming_message_callback, this, std::placeholders::_1));
}

void RelayNode::unsubscribe() {
  if (sub_) sub_.reset();
}

void RelayNode::incoming_message_callback(std::shared_ptr<rclcpp::SerializedMessage> msg) {
  rclcpp::SerializedMessage msg_rcl = *msg.get();
  pub_->publish(msg_rcl);
}

}  // namespace topic_tools


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<topic_tools::RelayNode>());
  rclcpp::shutdown();
  return 0;
}