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

from geometry_msgs.msg import Twist

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

from tf2_ros import LookupException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from turtlesim.srv import Spawn


class FrameListener(Node):

    def __init__(self):
        super().__init__('turtle_tf2_frame_listener')

        # Declare and acquire `target_frame` parameter
        self.declare_parameter('target_frame', 'turtle1')
        self.target_frame = self.get_parameter(
            'target_frame').get_parameter_value().string_value

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Create a client to spawn a turtle
        self.client = self.create_client(Spawn, 'spawn')

        # Check if the service is available
        while not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('service not available, waiting again...')

        # Initialize request with turtle name and coordinates
        # Note that x, y and theta are defined as floats in turtlesim/srv/Spawn
        request = Spawn.Request()
        request.name = 'turtle2'
        request.x = float(4)
        request.y = float(2)
        request.theta = float(0)
        # Call request
        self.client.call_async(request)

        # Create turtle2 velocity publisher
        self.turtle_vel_ = self.create_publisher(Twist, 'turtle2/cmd_vel', 1)

        # Call on_timer function every second
        self._output_timer = self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        # Store frame names in variables that will be used to
        # compute transformations
        from_frame_rel = self.target_frame
        to_frame_rel = 'turtle2'

        # Look up for the transformation between target_frame and turtle2 frames
        # and send velocity commands for turtle2 to reach target_frame
        try:
            now = rclpy.time.Time()
            trans = self._tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                now,
                timeout=Duration(seconds=1.0))
        except LookupException:
            self.get_logger().info('transform not ready')
            return

        msg = Twist()
        msg.angular.z = 1.0 * math.atan2(
            trans.transform.translation.y,
            trans.transform.translation.x)

        msg.linear.x = 0.5 * math.sqrt(
            trans.transform.translation.x ** 2 +
            trans.transform.translation.y ** 2)

        self.turtle_vel_.publish(msg)


def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
