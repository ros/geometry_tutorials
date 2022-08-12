# Copyright 2021 Open Source Robotics Foundation, Inc.
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

import math
import sys

from geometry_msgs.msg import TransformStamped

import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


# This function is a stripped down version of the code in
# https://github.com/matthew-brett/transforms3d/blob/f185e866ecccb66c545559bc9f2e19cb5025e0ab/transforms3d/euler.py
# Besides simplifying it, this version also inverts the order to return x,y,z,w, which is
# the way that ROS prefers it.
def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q


class StaticFramePublisher(Node):
    """
    Broadcast transforms that never change.

    This example publishes transforms from `world` to a static turtle frame.
    The transforms are only published once at startup, and are constant for all
    time.
    """

    def __init__(self, transformation):
        super().__init__('static_turtle_tf2_broadcaster')

        self._tf_publisher = StaticTransformBroadcaster(self)

        # Publish static transforms once at startup
        self.make_transforms(transformation)

    def make_transforms(self, transformation):
        static_transformStamped = TransformStamped()
        static_transformStamped.header.stamp = self.get_clock().now().to_msg()
        static_transformStamped.header.frame_id = 'world'
        static_transformStamped.child_frame_id = sys.argv[1]
        static_transformStamped.transform.translation.x = float(sys.argv[2])
        static_transformStamped.transform.translation.y = float(sys.argv[3])
        static_transformStamped.transform.translation.z = float(sys.argv[4])
        quat = quaternion_from_euler(
            float(sys.argv[5]), float(sys.argv[6]), float(sys.argv[7]))
        static_transformStamped.transform.rotation.x = quat[0]
        static_transformStamped.transform.rotation.y = quat[1]
        static_transformStamped.transform.rotation.z = quat[2]
        static_transformStamped.transform.rotation.w = quat[3]

        self._tf_publisher.sendTransform(static_transformStamped)


def main():
    logger = rclpy.logging.get_logger('logger')

    # obtain parameters from command line arguments
    if len(sys.argv) < 8:
        logger.info('Invalid number of parameters. Usage: \n'
                    '$ ros2 run turtle_tf2_py static_turtle_tf2_broadcaster'
                    'child_frame_name x y z roll pitch yaw')
        sys.exit(0)
    else:
        if sys.argv[1] == 'world':
            logger.info('Your static turtle name cannot be "world"')
            sys.exit(0)

    # pass parameters and initialize node
    rclpy.init()
    node = StaticFramePublisher(sys.argv)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
