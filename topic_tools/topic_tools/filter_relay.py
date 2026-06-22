#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2025 Alon Nusem
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

@author: Alon Nusem
Allows to republish messages on a different topic based on a matching filter.
* Examples
$ ros2 run topic_tools filter_relay /chatter /one "m.data == 1" /two "m.data == 2"
"""  # noqa

import argparse
import importlib
import os
import sys

import rclpy
from rclpy.utilities import remove_ros_args
from ros2topic.api import get_msg_class

from .tool_base_node import ToolBase, add_qos_args

class FilterRelay(ToolBase):

    def __init__(self, args, rules):
        super().__init__(f'filter_relay_{os.getpid()}')

        input_class = get_msg_class(
            self, args.input, blocking=args.wait_for_start, include_hidden_topics=True)

        if input_class is None:
            raise RuntimeError(f'ERROR: Wrong input topic: {args.input}')
        if len(rules) % 2 != 0 :
            raise RuntimeError(f'ERROR: Odd number of extra args, format should come in "topic" "filter" pairs: {args.input}')
        
        self.modules = {}
        for module in args.modules:
            try:
                mod = importlib.import_module(module)
            except ImportError:
                print(f'Failed to import module: {module}', file=sys.stderr)
            else:
                self.modules[module] = mod

        qos_profile = self.choose_qos(args, args.input)

        self.pubs = []
        self.filters = []
        for topic, filter in zip(rules[::2], rules[1::2]):
            try:
                self.filters.append(eval(f'lambda m: {filter}', self.modules))
            except NameError as e:
                print(f"Expression using variables other than 'm': {e.message}", file=sys.stderr)
                raise
            except UnboundLocalError as e:
                print(f'Wrong expression: {e.message}', file=sys.stderr)
                raise
            except Exception:
                raise
            self.pubs.append(self.create_publisher(input_class, topic, qos_profile))

        self.sub = self.create_subscription(
            input_class, args.input, self.callback, qos_profile)

    def callback(self, m):
        for filter, publisher in zip(self.filters, self.pubs):
            try:
                match = filter(m)
            except Exception:
                raise
            if match:
                publisher.publish(m)

def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(
        formatter_class=argparse.RawTextHelpFormatter,
        description=(
            'Allows to relay messages from one topic to others based on filter rules.'
            'Usage:\n\tros2 run topic_tools transform '
            '<input topic> <output topic> <output type> '
            '[<expression on m>] [--import numpy tf] [--field <topic_field>]\n\n'
            'Example:\n\tros2 run topic_tools filter_relay /chatter '
            '/one "m.data == 1" /two "m.data == 2"')
        )
    parser.add_argument('input', help='Input topic or topic field.')
    parser.add_argument(
        '-i', '--import', dest='modules', nargs='+', default=['numpy'],
        help='List of Python modules to import.'
    )
    parser.add_argument(
        '--wait-for-start', action='store_true',
        help='Wait for input messages.'
        )
    add_qos_args(parser)

    # get and strip out ros args first
    args, rules = parser.parse_known_args(remove_ros_args(args=argv))
    rclpy.init(args=argv)
    node = FilterRelay(args, rules)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('relay_field stopped cleanly')
    except BaseException:
        print('exception in relay_field:', file=sys.stderr)
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()
