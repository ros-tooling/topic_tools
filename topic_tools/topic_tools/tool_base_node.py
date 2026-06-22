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

from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSPresetProfiles
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSReliabilityPolicy
from ros2topic.api import qos_profile_from_short_keys


class ToolBase(Node):

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

def add_qos_args(parser):
    parser.add_argument(
        '--qos-profile',
        choices=QoSPresetProfiles.short_keys(),
        help='Quality of service preset profile to subscribe with (default: sensor_data)'
    )
    default_profile = QoSPresetProfiles.get_from_short_key('sensor_data')
    parser.add_argument(
        '--qos-depth', metavar='N', type=int,
        help='Queue size setting to subscribe with '
             '(overrides depth value of --qos-profile option)')
    parser.add_argument(
        '--qos-history',
        choices=QoSHistoryPolicy.short_keys(),
        help='History of samples setting to subscribe with '
             '(overrides history value of --qos-profile option, default: {})'
             .format(default_profile.history.short_key))
    parser.add_argument(
        '--qos-reliability',
        choices=QoSReliabilityPolicy.short_keys(),
        help='Quality of service reliability setting to subscribe with '
             '(overrides reliability value of --qos-profile option, default: '
             'Automatically match existing publishers )')
    parser.add_argument(
        '--qos-durability',
        choices=QoSDurabilityPolicy.short_keys(),
        help='Quality of service durability setting to subscribe with '
             '(overrides durability value of --qos-profile option, default: '
             'Automatically match existing publishers )')
