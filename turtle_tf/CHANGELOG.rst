^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package turtle_tf
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.2 (2015-03-03)
------------------
* remove old roslib invocations
* Contributors: Tully Foote

0.2.1 (2014-12-17)
------------------
* continuing on error instead of executing the rest of the block. Fixes `#13 <https://github.com/ros/geometry_tutorials/issues/13>`_
* merging conflicting pull requests
* catch tf.ExtrapolationException
  see http://answers.ros.org/question/12024/tftutorials-demo-problem-ros-electric/, this still happens in hydro
* Fixes for "Using sensor messages with tf" tutorial.
* add start_debug_demo.launch, that is introduced in http://wiki.ros.org/tf/Tutorials/Debugging%20tf%20problems
* Fix wrong order of arguments to q.setRPY in commit 82964f6.
* Fix wrong order of arguments to q.setRPY in commit 82964f6.
  The API shows the order is:
  void    setRPY (const tfScalar &roll, const tfScalar &pitch, const tfScalar &yaw)
  Line 14 should read:
  q.setRPY(0, 0, msg->theta);
  This fault breaks this tutorial:
  http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20%28C%2B%2B%29
  In a way that doesn't show up until the subsequent tutorial, where the
  follower doesn't follow. The fix to line 14 proposed above fixes the
  subsequent tutorial.
  The problem is discussed in this thread:
  http://answers.ros.org/question/45884/turtle-tf-tutorial-fails-to-work/?answer=47845#post-id-47845
  and an incorrect fix was applied to the wiki. The note in the wiki under
  the code, which says the code should read:
  transform.setRotation( tf::Quaternion(0, 0, msg->theta) );
  is wrong - it should say transform.setRPY.
* Added queue size for Publisher to avoid warning.
* add queue_size argument to rospy.Publisher
  `ros/ros_comm#169 <https://github.com/ros/ros_comm/issues/169>`_
* add queue_size argument to rospy publisher
  `ros/ros_comm#169 <https://github.com/ros/ros_comm/issues/169>`_
* add queue_size
  `ros/ros_comm#169 <https://github.com/ros/ros_comm/issues/169>`_
* Adding queue_size argument to publisher
  `ros/ros_comm#169 <https://github.com/ros/ros_comm/issues/169>`_
* Contributors: Denis Štogl, K.mura, Kei Okada, Paul Bouchier, Tully Foote, spourmehr

0.2.0 (2013-06-28 14:14:25 -0700)
---------------------------------
- Fixed catkinized
- Updated dependency on turtlesim, turtlesim switched to using geometry_msgs/Twist from turtlesim/Velocity msgs
