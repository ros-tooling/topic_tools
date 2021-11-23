#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2021 Daisuke Nishimatsu
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Usage summary.

@author: enriquefernandez, Daisuke Nishimatsu
Allows to take a topic or one of it fields and output it on another topic
after performing a valid python operation.
The operations are done on the message, which is taken in the variable 'm'.
* Examples (note that numpy is imported by default):
$ ros2 run topic_tools transform /imu --field orientation.x /x_str std_msgs/String 'std_msgs.msg.String(data=str(m))' --import std_msgs # noqa: E501
$ ros2 run topic_tools transform /imu --field orientation.x /x_in_degrees std_msgs/Float64 'std_msgs.msg.Float64(data=-numpy.rad2deg(m))' --import std_msgs numpy # noqa: E501
$ ros2 run topic_tools transform /imu --field orientation /norm std_msgs/Float64 'std_msgs.msg.Float64(data=numpy.sqrt(numpy.sum(numpy.array([m.x, m.y, m.z, m.w]))))' --import std_msgs numpy # noqa: E501
$ ros2 run topic_tools transform /imu --field orientation /norm std_msgs/Float64 'std_msgs.msg.Float64(data=numpy.linalg.norm([m.x, m.y, m.z, m.w]))' --import std_msgs numpy # noqa: E501
"""

import argparse
import importlib
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSPresetProfiles
from rclpy.qos import QoSReliabilityPolicy
from rclpy.utilities import remove_ros_args
from ros2topic.api import get_msg_class
from ros2topic.api import qos_profile_from_short_keys
from rosidl_runtime_py.utilities import get_message


class Transform(Node):

    def __init__(self, args):
        super().__init__(f'transform_{os.getpid()}')

        self.modules = {}
        for module in args.modules:
            try:
                mod = importlib.import_module(module)
            except ImportError:
                print(f'Failed to import module: {module}', file=sys.stderr)
            else:
                self.modules[module] = mod

        self.expression = args.expression

        input_topic_in_ns = args.input
        if not input_topic_in_ns.startswith('/'):
            input_topic_in_ns = self.get_namespace() + args.input

        input_class = get_msg_class(
            self, input_topic_in_ns, blocking=args.wait_for_start, include_hidden_topics=True)

        if input_class is None:
            raise RuntimeError(f'ERROR: Wrong input topic: {input_topic_in_ns}')

        self.field = args.field
        if self.field is not None:
            self.field = list(filter(None, self.field.split('.')))
            if not self.field:
                raise RuntimeError(f"Invalid field value '{args.field}'")

        self.output_class = get_message(args.output_type)

        qos_profile = self.choose_qos(args, input_topic_in_ns)

        self.pub = self.create_publisher(self.output_class, args.output_topic, qos_profile)
        self.sub = self.create_subscription(
            input_class, input_topic_in_ns, self.callback, qos_profile)

    def choose_qos(self, args, topic_name):

        if (args.qos_profile is not None or
                args.qos_reliability is not None or
                args.qos_durability is not None or
                args.qos_depth is not None or
                args.qos_history is not None):

            if args.qos_profile is None:
                args.qos_profile = 'sensor_data'
            return qos_profile_from_short_keys(args.qos_profile,
                                               reliability=args.qos_reliability,
                                               durability=args.qos_durability,
                                               depth=args.qos_depth,
                                               history=args.qos_history)

        qos_profile = QoSPresetProfiles.get_from_short_key('sensor_data')
        reliability_reliable_endpoints_count = 0
        durability_transient_local_endpoints_count = 0

        pubs_info = self.get_publishers_info_by_topic(topic_name)
        publishers_count = len(pubs_info)
        if publishers_count == 0:
            return qos_profile

        for info in pubs_info:
            if (info.qos_profile.reliability == QoSReliabilityPolicy.RELIABLE):
                reliability_reliable_endpoints_count += 1
            if (info.qos_profile.durability == QoSDurabilityPolicy.TRANSIENT_LOCAL):
                durability_transient_local_endpoints_count += 1

        # If all endpoints are reliable, ask for reliable
        if reliability_reliable_endpoints_count == publishers_count:
            qos_profile.reliability = QoSReliabilityPolicy.RELIABLE
        else:
            if reliability_reliable_endpoints_count > 0:
                print(
                    'Some, but not all, publishers are offering '
                    'QoSReliabilityPolicy.RELIABLE. Falling back to '
                    'QoSReliabilityPolicy.BEST_EFFORT as it will connect '
                    'to all publishers'
                )
            qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

        # If all endpoints are transient_local, ask for transient_local
        if durability_transient_local_endpoints_count == publishers_count:
            qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        else:
            if durability_transient_local_endpoints_count > 0:
                print(
                    'Some, but not all, publishers are offering '
                    'QoSDurabilityPolicy.TRANSIENT_LOCAL. Falling back to '
                    'QoSDurabilityPolicy.VOLATILE as it will connect '
                    'to all publishers'
                )
            qos_profile.durability = QoSDurabilityPolicy.VOLATILE

        return qos_profile

    def callback(self, m):
        if self.field is not None:
            for field in self.field:
                try:
                    m = getattr(m, field)
                except AttributeError as ex:
                    raise RuntimeError(f"Invalid field '{'.'.join(self.field)}': {ex}")
        try:
            res = eval(f'{self.expression}', self.modules, {'m': m})
        except NameError as e:
            print(f"Expression using variables other than 'm': {e.message}", file=sys.stderr)
        except UnboundLocalError as e:
            print(f'Wrong expression: {e.message}', file=sys.stderr)
        except Exception:
            raise
        else:
            if not isinstance(res, (list, tuple)):
                res = [res]
            self.pub.publish(*res)


def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(
        formatter_class=argparse.RawTextHelpFormatter,
        description='Apply a Python operation to a topic.\n\n'
                    'A node is created that subscribes to a topic,\n'
                    'applies a Python expression to the topic (or topic\n'
                    'field) message \"m\", and publishes the result\n'
                    'through another topic.\n\n'
                    'Usage:\n\tros2 run topic_tools transform '
                    '<input topic> <output topic> <output type> '
                    '[<expression on m>] [--import numpy tf] [--field <topic_field>]\n\n'
                    'Example:\n\tros2 run topic_tools transform /imu --field orientation '
                    '/norm std_msgs/Float64'
                    '\"std_msgs.msg.Float64(data=sqrt(sum(array([m.x, m.y, m.z, m.w]))))\"'
                    ' --import std_msgs')
    parser.add_argument('input', help='Input topic or topic field.')
    parser.add_argument('output_topic', help='Output topic.')
    parser.add_argument('output_type', help='Output topic type.')
    parser.add_argument(
        'expression', default='m',
        help='Python expression to apply on the input message \"m\".'
    )
    parser.add_argument(
        '-i', '--import', dest='modules', nargs='+', default=['numpy'],
        help='List of Python modules to import.'
    )
    parser.add_argument(
        '--wait-for-start', action='store_true',
        help='Wait for input messages.'
    )
    parser.add_argument(
        '--qos-profile',
        choices=rclpy.qos.QoSPresetProfiles.short_keys(),
        help='Quality of service preset profile to subscribe with (default: sensor_data)'
    )
    default_profile = rclpy.qos.QoSPresetProfiles.get_from_short_key('sensor_data')
    parser.add_argument(
        '--qos-depth', metavar='N', type=int,
        help='Queue size setting to subscribe with '
             '(overrides depth value of --qos-profile option)')
    parser.add_argument(
        '--qos-history',
        choices=rclpy.qos.QoSHistoryPolicy.short_keys(),
        help='History of samples setting to subscribe with '
             '(overrides history value of --qos-profile option, default: {})'
             .format(default_profile.history.short_key))
    parser.add_argument(
        '--qos-reliability',
        choices=rclpy.qos.QoSReliabilityPolicy.short_keys(),
        help='Quality of service reliability setting to subscribe with '
             '(overrides reliability value of --qos-profile option, default: '
             'Automatically match existing publishers )')
    parser.add_argument(
        '--qos-durability',
        choices=rclpy.qos.QoSDurabilityPolicy.short_keys(),
        help='Quality of service durability setting to subscribe with '
             '(overrides durability value of --qos-profile option, default: '
             'Automatically match existing publishers )')
    parser.add_argument(
        '--field', type=str, default=None,
        help='Transform a selected field of a message. '
             "Use '.' to select sub-fields. "
             'For example, to transform the orientation x field of a sensor_msgs/msg/Imu message: '
             "'ros2 run topic_tools transform /imu --field orientatin.x'",
    )
    args = parser.parse_args(remove_ros_args(args=argv))
    rclpy.init(args=argv)

    node = Transform(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('transform stopped cleanly')
    except BaseException:
        print('exception in transform:', file=sys.stderr)
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()
