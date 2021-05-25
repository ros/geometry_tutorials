import math
import sys
import transforms3d
import rclpy

from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from turtlesim.msg import Pose
import numpy

class FramePublisher(Node):
    def __init__(self, turtlename):
        super().__init__('frame_publisher')
        self.turtlename = turtlename
        self.subscription = self.create_subscription(
            Pose,
            '/%s/pose' % turtlename,
            self.handle_turtle_pose,
            10)
        self.subscription 

    def handle_turtle_pose(self, msg):
        # self.get_logger().info('I heard: "%s"' % msg)
        br = TransformBroadcaster(self)
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = self.turtlename
        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0
        q = transforms3d.taitbryan.euler2quat(
                    msg.theta, 0, 0)
        t.transform.rotation.x = q[1]
        t.transform.rotation.y = q[2]
        t.transform.rotation.z = q[3]
        t.transform.rotation.w = q[0]

        # self.get_logger().info('TB {}'.format(repr([q1[1], q1[2], q1[3], q1[0]])))
        self.get_logger().info(repr(t.transform))
        br.sendTransform(t)

def main():
    rclpy.init()
    turtlename = "turtle1"

    # print(self.get_parameter('some_int')._value)

    node = FramePublisher(turtlename)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
