#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy

import math
import tf
import geometry_msgs.msg
import turtlesim.srv

if __name__ == '__main__':
    rospy.init_node('turtle_tf_listener')

    listener = tf.TransformListener()

    rospy.wait_for_service('spawn')
    spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    spawner(4, 2, 0, 'turtle2')

    turtle_vel = rospy.Publisher('turtle2/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)

    turtle1_pose = geometry_msgs.msg.PoseStamped()
    turtle1_pose.header.frame_id = "turtle1"
    turtle1_pose.pose.orientation.w = 1.0  # Neutral orientation
    turtle1_pose_in_turtle2_frame = geometry_msgs.msg.PoseStamped()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform('/turtle2', '/turtle1', rospy.Time())
            turtle1_pose_in_turtle2_frame = listener.transformPose('turtle2', turtle1_pose)
            ### None of the below alternatives work
            # turtle1_pose_in_turtle2_frame = listener.transformPose("/turtle2", rospy.Time(), turtle1_pose, "world", turtle1_pose_in_turtle2_frame)
            # turtle1_pose_in_turtle2_frame = listener.transformPose("/turtle2", rospy.Time(), turtle1_pose, "world")
            # turtle1_pose_in_turtle2_frame = listener.transformPose("/turtle2", rospy.Time(), turtle1_pose)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        angular = 4 * math.atan2(trans[1], trans[0])
        linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        msg = geometry_msgs.msg.Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        turtle_vel.publish(msg)

        # rospy.loginfo("Turtle2 sees turtle1 at x: %2f, y:  %2f", turtle1_pose_in_turtle2_frame.pose.position.x, turtle1_pose_in_turtle2_frame.pose.position.y)

        rate.sleep()
