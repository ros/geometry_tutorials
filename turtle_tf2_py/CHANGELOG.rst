^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package turtle_tf2_py
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.6 (2022-09-15)
------------------
* Minor cleanups across the tutorials. (`#71 <https://github.com/ros/geometry_tutorials/issues/71>`_)
* Contributors: Chris Lalancette

0.3.5 (2022-09-08)
------------------
* Remove the dependency on tf_transformations. (`#69 <https://github.com/ros/geometry_tutorials/issues/69>`_)
* Contributors: Chris Lalancette

0.3.4 (2021-10-11)
------------------
* Add source code and launch file for tf2 PointStamped message publisher and listener/filter (`#62 <https://github.com/ros/geometry_tutorials/issues/62>`_)
  Add Python Code of PointStamped Messages Broadcaster Node
  Add launch File turtle_tf2_sensor_message.launch.py
  Update CMakeLists.txt for turtle_tf2_message_filter node
  Update package.xml for dependencies of turtle_tf2_message_filter.cpp
  Update turtle_tf2_message_filter.cpp after checking linters.
  for the copyright year from 2015 to 2021 and delete the line "self.sub".
  sort include headers; change two node instances to one instance; change turtle3 spawning manner.
  remove whitespaces in line 39 and 42
  some little updates
* Contributors: kenny_wang

0.3.3 (2021-08-27)
------------------
* update python tf2 listener node (`#58 <https://github.com/ros/geometry_tutorials/issues/58>`_)
* restructure code in static transform broadcaster (`#57 <https://github.com/ros/geometry_tutorials/issues/57>`_)
* move TransformBroadcaster to the init call to resolve memory leak (`#56 <https://github.com/ros/geometry_tutorials/issues/56>`_)
* Contributors: kurshakuz

0.3.2 (2021-08-09)
------------------
* Add fixed and dynamic frame broadcaster nodes (`#40 <https://github.com/ros/geometry_tutorials/issues/40>`_)
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
* Add target frame parameter to the listener node (`#41 <https://github.com/ros/geometry_tutorials/issues/41>`_)
* fix linter style issues and remove spin_thread=False statement (`#39 <https://github.com/ros/geometry_tutorials/issues/39>`_)
* Contributors: kurshakuz

0.3.1 (2021-06-23)
------------------
* Replace dependency on transforms3d pip package to tf_transformations ROS2 package (`#38 <https://github.com/ros/geometry_tutorials/issues/38>`_)
* Contributors: kurshakuz

0.0.3
-----
* Add Audrow as a maintainer and move Shyngys to author (`#37 <https://github.com/ros/geometry_tutorials/issues/37>`_)
* Fix linters (`#35 <https://github.com/ros/geometry_tutorials/issues/35>`_)
* Migrate turtle_tf2 tutorial package to ROS2 (`#34 <https://github.com/ros/geometry_tutorials/issues/34>`_)
* Contributors: Alejandro Hernández Cordero, kurshakuz, Audrow Nash
