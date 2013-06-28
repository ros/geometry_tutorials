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
#!/usr/bin/env python

import roslib
roslib.load_manifest('turtle_tf')
import rospy

import turtlesim.msg
import geometry_msgs.msg
import turtlesim.srv
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Header


class PointPublisher:
    def handle_turtle_pose(self, msg, turtlename):
        self.pub.publish(PointStamped(Header(0, rospy.rostime.get_rostime(), "/world"), Point(msg.x, msg.y, 0)))

    def __init__(self):
        self.turtlename = "turtle3"  # rospy.get_param('~turtle')
        self.sub = rospy.Subscriber('/%s/pose' % self.turtlename,
                                    turtlesim.msg.Pose,
                                    self.handle_turtle_pose,
                                    self.turtlename)
        self.pub = rospy.Publisher('turtle_point_stamped', PointStamped)

if __name__ == '__main__':
    rospy.init_node('tf_turtle_stamped_msg_publisher')
    rospy.wait_for_service('spawn')
    spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    spawner(4, 2, 0, 'turtle3')

    pp = PointPublisher()

    pub = rospy.Publisher("turtle3/cmd_vel", geometry_msgs.msg.Twist)
    while not rospy.is_shutdown():
        msg = geometry_msgs.msg.Twist()
        msg.linear.x = 1
        msg.angular.z = 1
        pub.publish(msg)
        rospy.sleep(rospy.Duration(0.1))
