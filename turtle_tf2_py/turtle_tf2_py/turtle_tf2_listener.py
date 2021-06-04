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

import rclpy
from rclpy.node import Node
import rclpy.time
from tf2_ros import LookupException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from turtlesim.srv import Spawn
from geometry_msgs.msg import Twist
import math
from rclpy.duration import Duration


class FrameListener(Node):
    def __init__(self):
        super().__init__('turtle_tf2_frame_listener')

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)

        # Create a client to spawn a turtle
        self.client = self.create_client(Spawn, 'spawn')

        # Check if the service is available
        while not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('service not available, waiting again...')

        # Initialize request with turtle name and coordinates
        request = Spawn.Request()
        request.name = "turtle2"
        request.x = float(4)
        request.y = float(2)
        request.theta = float(0)
        # Call request
        self.client.call_async(request)

        # Crete turtle2 velocity publisher
        self.turtle_vel_ = self.create_publisher(Twist, 'turtle2/cmd_vel', 1)

        # Call on_timer function every second
        self._output_timer = self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        from_frame_rel = 'turtle1'
        to_frame_rel = 'turtle2'

        # Look up for the transformation between turtle1 and turtle2 frames
        # and send velocity commands for turtle2 to reach turtle1
        try:
            when = rclpy.time.Time()
            trans = self._tf_buffer.lookup_transform(to_frame_rel, from_frame_rel, when, timeout=Duration(seconds=1.0))

            msg = Twist()
            msg.angular.z = 1.0 * math.atan2(trans.transform.translation.y, trans.transform.translation.x)
            msg.linear.x = 0.5 * math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)
            self.turtle_vel_.publish(msg)
        except LookupException:
            self.get_logger().info('transform not ready')


def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
