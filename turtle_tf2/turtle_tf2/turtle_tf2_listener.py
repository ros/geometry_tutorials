
import rclpy
from rclpy.node import Node
import rclpy.time
from tf2_ros import LookupException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from turtlesim.srv import Spawn
from geometry_msgs.msg import Twist
import math
import numpy
from rclpy.duration import Duration

class FrameListener(Node):
    def __init__(self):
        super().__init__('frame_listener')

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)

        # Spawn turtle
        self.client = self.create_client(Spawn, 'spawn')
        # Check if the a service is available  
        while not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('service not available, waiting again...')
        request = Spawn.Request()
        request.name = "turtle2"
        request.x = float(4)
        request.y = float(2)
        request.theta = float(0)
        future = self.client.call_async(request)

        self.turtle_vel_ = self.create_publisher(Twist, 'turtle2/cmd_vel', 1)
        self._output_timer = self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        from_frame_rel = 'turtle1'
        to_frame_rel = 'turtle2'

        try:
            when = rclpy.time.Time()
            trans = self._tf_buffer.lookup_transform(
                to_frame_rel, from_frame_rel, when, timeout=Duration(seconds=1.0))

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
