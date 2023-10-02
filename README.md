# topic_tools

This package is the ROS 2 port of https://wiki.ros.org/topic_tools

Tools for directing, throttling, selecting, and otherwise manipulating ROS 2 topics at a meta-level. These tools do not generally perform serialization on the streams being manipulated, instead acting on generic binary data using `rclcpp`'s `GenericPublisher` and `GenericSubscription`.

The tools in this package are provided as composable ROS 2 component nodes, so that they can be spawned into an existing process, launched from launchfiles, or invoked from the command line.

## Components

- [Relay](#relay): Subscribes to a topic and republishes to another.
- [RelayField](#relayfield): Republishes data in a different message type.
- [Transform](#transform): Manipulates a topic or a field and outputs data on another topic.
- [Throttle](#throttle): Republishes data with bandwidth or rate limit.
- [Drop](#drop): Republishes by dropping X out of every Y incoming messages.
- [Mux](#mux): Multiplexes incoming topics to an output.
- [Delay](#delay): Delays and republishes incoming data.

### Relay

Relay is ROS 2 node that subscribes to a topic and republishes all incoming data to another topic. It can work with any message type.

#### Usage

```shell
ros2 run topic_tools relay <intopic> [outtopic]
```

Subscribe to `intopic` and republish to either
- `outtopic` if specified
- `<intopic>_relay` if not

E.g. rename `base_scan` to `my_base_scan`:

```shell
ros2 run topic_tools relay base_scan my_base_scan
```

#### Parameters

- `input_topic` (string)
    - the same as if provided as a command line argument
- `output_topic` (string, default=`<input_topic>_relay`)
    - the same as if provided as a command line argument
- `lazy` (bool, default=False)
    - If True, only subscribe to `input_topic` if there is at least one subscriber on the `output_topic`

### RelayField

RelayField is a ROS 2 node that allows to republish data in a different message type.

#### Usage

```shell
ros2 run topic_tools relay_field <input topic> <output topic> <output type> [<expression on m>]
```

Subscribe to `input topic` and republish one or many of its fields onto another field in a different message type

E.g. publish the contents of the `data` field in a `std_msgs/msg/String` onto the `frame_id` field of a `std_msgs/msg/Header`:

```shell
ros2 run topic_tools relay_field /chatter /header std_msgs/Header "{stamp: {sec: 0, nanosec: 0}, frame_id: m.data}"
```

### Transform

Transform is a ROS 2 node that allows to take a topic or one of it fields and output it on another topic.

#### Usage

```shell
ros2 run topic_tools transform <input topic> <output topic> <output type> [<expression on m>] [--import <modules>] [--field <topic_field>]
```

Subscribe to `input topic` and convert topic content or its field into
- `output topic` whose type is `output type` based on `expression on m`

E.g. transform `imu` orientation to `norm`:

```shell
ros2 run topic_tools transform /imu --field orientation /norm std_msgs/Float64 'std_msgs.msg.Float64(data=numpy.sqrt(numpy.sum(numpy.array([m.x, m.y, m.z, m.w]))))' --import std_msgs numpy
```

### Throttle

Throttle is ROS 2 node that subscribes to a topic and republishes incoming data to another topic, either at a maximum bandwidth or maximum message rate.

#### Usage

#### throttle message (rate)

```shell
ros2 run topic_tools throttle messages <intopic> <msgs_per_sec> [outtopic]
```

Throttle messages on `intopic` to a particular rate.
- `intopic`: Incoming topic to subscribe to
- `msgs_per_sec`: Maximum messages per second to let through.
- `outtopic`: Outgoing topic to publish on (default: intopic_throttle)

E.g. throttle bandwidth-hogging laser scans (base_scan) to 1Hz:

```shell
ros2 run topic_tools throttle messages base_scan 1.0
```

#### throttle bytes (bandwidth)

```shell
ros2 run topic_tools throttle bytes <intopic> <bytes_per_sec> <window> [outtopic]
```

Throttle messages on `intopic` to a particular rate.
- `intopic`: Incoming topic to subscribe to
- `bytes_per_sec`: Maximum bytes of messages per second to let through.
- `window`: Time window in seconds to consider
- `outtopic`: Outgoing topic to publish on (default: intopic_throttle)

E.g. throttle bandwidth-hogging laser scans (base_scan) to 1KBps:

```shell
ros2 run topic_tools throttle bytes base_scan 1024 1.0
```

#### Parameters

- `input_topic` (string)
    - the same as if provided as a command line argument
- `output_topic` (string, default=`<input_topic>_throttle`)
    - the same as if provided as a command line argument
- `lazy` (bool, default=False)
    - If True, only subscribe to `input_topic` if there is at least one subscriber on the `output_topic`
- `use_wall_clock` (bool, default=False)
    - If True, then perform all rate measurements against wall clock time, regardless of whether simulation / log time is in effect.
- `throttle_type` (string, either `messages` or `bytes`)
    - Method how to throttle
- `msgs_per_sec` (double)
    - Maximum messages per second to let through.
- `bytes_per_sec` (integer)
    - Maximum bytes of messages per second to let through.
- `window` (double)
    - Time window in seconds to consider

### Drop

Drop is ROS 2 node that can subscribe to a topic and republish incoming data to another topic, dropping X out of every Y incoming messages.
It's mainly useful for limiting bandwidth usage, e.g., over a wireless link. It can work with any message type.

#### Usage

```shell
ros2 run topic_tools drop <intopic> <X> <Y> [outtopic]
```

Subscribe to <intopic> and drop every <X> out of <Y> messages.
- `intopic`: Incoming topic to subscribe to
- `X`, `Y`: drop X out of every Y incoming messages
- `outtopic`: Outgoing topic to publish on (default: `intopic`_drop, e.g. when intopic is "base_scan", then it will be base_scan_drop)

E.g. drop every other message published to base_scan:

```shell
ros2 run topic_tools drop base_scan 1 2
```

#### Parameters

- `input_topic` (string)
    - the same as if provided as a command line argument
- `output_topic` (string, default=`<input_topic>_drop`)
    - the same as if provided as a command line argument
- `lazy` (bool, default=False)
    - If True, only subscribe to `input_topic` if there is at least one subscriber on the `output_topic`
- `X` (int), `Y` (int)
    - Drop X out of every Y incoming messages

### Mux

mux is a ROS2 node that subscribes to a set of incoming topics and republishes incoming data from one of them to another topic,
i.e., it's a multiplexer that switches an output among 1 of N inputs. Services are offered to switch among input topics,
and to add and delete input topics. At startup, the first input topic on the command line is selected.

#### Usage

```shell
ros2 run topic_tools mux <outopic> <intopic1> [intopic2...]
```

Subscribe to <intopic1>...N and publish currently selected topic to outopic. mux will start with <intopic1> selected.
- `outtopic`: Outgoing topic to publish on
- `intopicN`: Incoming topic to subscribe to

E.g. mux two command streams (auto_cmdvel and joystick_cmdvel) into one (sel_cmdvel):

```shell
ros2 run topic_tools mux sel_cmdvel auto_cmdvel joystick_cmdvel
```

#### Parameters

- `input_topics` (string array)
    - the same as if provided as a command line argument
- `output_topic` (string, default=`~/selected`)
    - the same as if provided as a command line argument
- `lazy` (bool, default=False)
    - If True, only subscribe to `input_topic` if there is at least one subscriber on the `output_topic`
- `initial_topic` (str, default="")
    - Input topic to select on startup. If `__none`, start with no input topic. If unset, default to first topic in arguments

### Delay

Delay is a ROS 2 node that can subscribe to a topic and republish incoming data to another topic, delaying the message by a fixed duration.
It's useful to simulate computational results with high latency.

#### Usage

```shell
ros2 run topic_tools delay <intopic> <delay> [outtopic]
```

Subscribe to <intopic> and republish on <outtopic> delayed by <delay>.
- `intopic`: Incoming topic to subscribe to
- `delay`: delay in seconds
- `outtopic`: Outgoing topic to publish on (default: intopic_delay)

E.g. delay messages published to base_scan by 500ms:

```shell
ros2 run topic_tools delay base_scan 0.5
```

#### Parameters

- `input_topic` (string)
    - the same as if provided as a command line argument
- `output_topic` (string, default=`<input_topic>_delay`)
    - the same as if provided as a command line argument
- `delay` (double, default=0.0)
    - delay in seconds
- `use_wall_clock` (bool, default=False)
    - If True, then perform all rate measurements against wall clock time, regardless of whether simulation / log time is in effect.
