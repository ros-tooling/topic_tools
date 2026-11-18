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
from rclpy.utilities import remove_ros_args
from ros2topic.api import get_msg_class
from rosidl_runtime_py.utilities import get_message

from .tool_base_node import ToolBase, add_qos_args

class Transform(ToolBase):

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

        try:
            self.expression_as_lambda = eval(f'lambda m: {self.expression}', self.modules)
        except NameError as e:
            print(f"Expression using variables other than 'm': {e.message}", file=sys.stderr)
            raise
        except UnboundLocalError as e:
            print(f'Wrong expression: {e.message}', file=sys.stderr)
            raise
        except Exception:
            raise

        input_class = get_msg_class(
            self, args.input, blocking=args.wait_for_start, include_hidden_topics=True)

        if input_class is None:
            raise RuntimeError(f'ERROR: Wrong input topic: {args.input}')

        self.field = args.field
        if self.field is not None:
            self.field = list(filter(None, self.field.split('.')))
            if not self.field:
                raise RuntimeError(f"Invalid field value '{args.field}'")

        self.output_class = get_message(args.output_type)

        qos_profile = self.choose_qos(args, args.input)

        self.pub = self.create_publisher(self.output_class, args.output_topic, qos_profile)
        self.sub = self.create_subscription(
            input_class, args.input, self.callback, qos_profile)

    def callback(self, m):
        if self.field is not None:
            for field in self.field:
                try:
                    m = getattr(m, field)
                except AttributeError as ex:
                    raise RuntimeError(f"Invalid field '{'.'.join(self.field)}': {ex}")
        try:
            res = self.expression_as_lambda(m)
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
    add_qos_args(parser)
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
