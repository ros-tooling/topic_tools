# topic_tools

This package is the ROS 2 port of http://wiki.ros.org/topic_tools

Tools for directing, throttling, selecting, and otherwise manipulating ROS 2 topics at a meta-level. These tools do not generally perform serialization on the streams being manipulated, instead acting on generic binary data using `rclcpp`'s `GenericPublisher` and `GenericSubscription`.

The tools in this package are provided as composable ROS 2 component nodes, so that they can be spawned into an existing process, launched from launchfiles, or invoked from the command line.

## Components

TBD
