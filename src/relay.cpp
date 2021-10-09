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
    rclcpp::GenericSubscription::SharedPtr sub_;
    rclcpp::GenericPublisher::SharedPtr pub_;
};

RelayNode::RelayNode() : rclcpp::Node("Relay") {
  std::string input_topic = declare_parameter<std::string>("input_topic");
  std::string output_topic = declare_parameter<std::string>("output_topic", input_topic + "_relay");
  std::string topic_type = declare_parameter<std::string>("type");
  pub_ = this->create_generic_publisher(output_topic, topic_type, rclcpp::QoS(1));
  sub_ = this->create_generic_subscription(
    input_topic, topic_type, rclcpp::QoS(1),
    [this](std::shared_ptr<rclcpp::SerializedMessage> msg) {
      rclcpp::SerializedMessage msg_rcl = *msg.get();
      pub_->publish(msg_rcl);
    });
}

}  // namespace topic_tools


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<topic_tools::RelayNode>());
  rclcpp::shutdown();
  return 0;
}