
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
        # print(self._tf_buffer.all_frames_as_yaml())
        from_frame_rel = 'turtle2'
        to_frame_rel = 'turtle1'

        from_frame_glob = 'turtle2'
        to_frame_glob = 'world'

        # self.get_logger().info('Waiting for transform from {} to {}'.format(from_frame, to_frame))
        try:
            # Block until the transform is available
            when = rclpy.time.Time()
            trans_rel = self._tf_buffer.lookup_transform(
                to_frame_rel, from_frame_rel, when, timeout=Duration(seconds=1.0))

            trans_glob = self._tf_buffer.lookup_transform(
                to_frame_glob, from_frame_glob, when, timeout=Duration(seconds=1.0))

            # self.get_logger().info('Got {}'.format(repr(trans)))

            angle_to_goal = math.atan2(trans_rel.transform.translation.y, trans_rel.transform.translation.x)
            self.get_logger().info('angle {}'.format(repr(angle_to_goal)))

            # theta = euler_from_quaternion([trans_glob.transform.rotation.x, trans_glob.transform.rotation.y, trans_glob.transform.rotation.z, trans_glob.transform.rotation.w])[2]
            # self.get_logger().info('theta {}'.format(repr(theta)))
            # self.get_logger().info('degree {}'.format(repr(math.degrees(theta))))
            # msg = Twist()
            # if abs(angle_to_goal - theta) > 0.1:
            #     msg.linear.x = 0.0
            #     msg.angular.z = 0.3
            # else:
            #     msg.linear.x = 0.5
            #     msg.angular.z = 0.0

            # msg.angular.z = 1.0 * math.atan2(trans.transform.translation.y, trans.transform.translation.x)
            # msg.linear.x = 0.5 * math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)
            # self.turtle_vel_.publish(msg)
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
