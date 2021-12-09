# topic_tools

This package is the ROS 2 port of http://wiki.ros.org/topic_tools

Tools for directing, throttling, selecting, and otherwise manipulating ROS 2 topics at a meta-level. These tools do not generally perform serialization on the streams being manipulated, instead acting on generic binary data using `rclcpp`'s `GenericPublisher` and `GenericSubscription`.

The tools in this package are provided as composable ROS 2 component nodes, so that they can be spawned into an existing process, launched from launchfiles, or invoked from the command line.

## Components

### Relay

Relay is ROS 2 node that subscribes to a topic and republishes all incoming data to another topic. It can work with any message type.

#### Usage

```
ros2 run topic_tools relay <intopic> [outtopic]
```

Subscribe to `intopic` and republish to either
- `outtopic` if specified
- `<intopic>_relay` if not

E.g. rename `base_scan` to `my_base_scan`:

```
ros2 run topic_tools relay base_scan my_base_scan
```

#### Parameters

- `input_topic` (string)
    - the same as if provided as a command line argument
- `output_topic` (string, default=`<input_topic>_relay`)
    - the same as if provided as a command line argument
- `lazy` (bool, default=False)
    - If True, only subscribe to `input_topic` if there is at least one subscriber on the `output_topic`

### Transform

Transform is ROS 2 node that allows to take a topic or one of it fields and output it on another topic

#### Usage

```
ros2 run topic_tools transform <input topic> <output topic> <output type> [<expression on m>] [--import <modules>] [--field <topic_field>]
```

Subscribe to `input topic` and convert topic content or its field into
- `output topic` whose type is `output type` based on `expression on m`

E.g. transform `imu` orientation to `norm`:

```
ros2 run topic_tools transform /imu --field orientation /norm std_msgs/Float64 'std_msgs.msg.Float64(data=numpy.sqrt(numpy.sum(numpy.array([m.x, m.y, m.z, m.w]))))' --import std_msgs numpy
```

### Throttle

Throttle is ROS 2 node that subscribes to a topic and republishes incoming data to another topic, either at a maximum bandwidth or maximum message rate.

#### Usage

#### throttle message (rate)

```
ros2 run topic_tools throttle messages <intopic> <msgs_per_sec> [outtopic]
```

Throttle messages on `intopic` to a particular rate.
- `intopic`: Incoming topic to subscribe to
- `msgs_per_sec`: maximum messages per second to let through.
- `outtopic`: Outgoing topic to publish on (default: intopic_throttle)

E.g. throttle bandwidth-hogging laser scans (base_scan) to 1Hz:

```
ros2 run topic_tools throttle messages base_scan 1.0
```

#### throttle bytes (bandwidth)

```
ros2 run topic_tools throttle bytes <intopic> <bytes_per_sec> <window> [outtopic]
```

Throttle messages on `intopic` to a particular rate.
- `intopic`: Incoming topic to subscribe to
- `msgs_per_sec`: maximum messages per second to let through.
- `outtopic`: Outgoing topic to publish on (default: intopic_throttle)

E.g. throttle bandwidth-hogging laser scans (base_scan) to 1KBps:

```
ros2 run topic_tools throttle bytes base_scan 1024 1.0
```

#### Parameters

- `input_topic` (string)
    - the same as if provided as a command line argument
- `output_topic` (string, default=`<input_topic>_throttle`)
    - the same as if provided as a command line argument
- `lazy` (bool, default=False)
    - If True, only subscribe to `input_topic` if there is at least one subscriber on the `output_topic`
- `ues_wall_clock` (bool, default=False)
    - If True, then perform all rate measurements against wall clock time, regardless of whether simulation / log time is in effect.

### Drop

Drop is ROS 2 node that can subscribe to a topic and republish incoming data to another topic, dropping X out of every Y incoming messages.
It's mainly useful for limiting bandwidth usage, e.g., over a wireless link. It can work with any message type.

#### Usage

```
ros2 run topic_tools drop <intopic> <X> <Y> [outtopic]
```

Subscribe to <intopic> and drop every <X> out of <Y> messages.
- `intopic`: Incoming topic to subscribe to
- `X`, `Y`: drop X out of every Y incoming messages
- `outtopic`: Outgoing topic to publish on (default: `intopic`_drop, e.g. when intopic is "base_scan", then it will be base_scan_drop)

E.g. drop every other message published to base_scan:

```
ros2 run topic_tools drop base_scan 1 2
```

#### Parameters

- `input_topic` (string)
    - the same as if provided as a command line argument
- `output_topic` (string, default=`<input_topic>_drop`)
    - the same as if provided as a command line argument
- `lazy` (bool, default=False)
    - If True, only subscribe to `input_topic` if there is at least one subscriber on the `output_topic`
- `X`, `Y` (int)
    - drop X out of every Y incoming messages
