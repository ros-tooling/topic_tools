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

#include "rclcpp/rclcpp.hpp"
#include "topic_tools/throttle_node.hpp"

int main(int argc, char * argv[])
{
  auto args = rclcpp::init_and_remove_ros_arguments(argc, argv);
  auto options = rclcpp::NodeOptions{};

  if (args.size() < 2) {
    std::cout << "Throttle type is missing." << std::endl;
    return 0;
  }
  if (args.at(1) == "messages") {
    options.append_parameter_override("throttle_type", "messages");
    if (args.size() >= 4) {
      options.append_parameter_override("input_topic", args.at(2));
      options.append_parameter_override("msgs_per_sec", std::stod(args.at(3)));
      if (args.size() >= 5) {
        options.append_parameter_override("output_topic", args.at(4));
      }
    }
  } else if (args.at(1) == "bytes") {
    options.append_parameter_override("throttle_type", "bytes");
    if (args.size() >= 5) {
      options.append_parameter_override("input_topic", args.at(2));
      options.append_parameter_override("bytes_per_sec", std::stoi(args.at(3)));
      options.append_parameter_override("window", std::stod(args.at(4)));
      if (args.size() >= 6) {
        options.append_parameter_override("output_topic", args.at(5));
      }
    }
  }

  auto node = std::make_shared<topic_tools::ThrottleNode>(options);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
