^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package turtle_tf2_cpp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.6 (2022-09-15)
------------------
* Minor cleanups across the tutorials. (`#71 <https://github.com/ros/geometry_tutorials/issues/71>`_)
* Contributors: Chris Lalancette

0.3.5 (2022-09-08)
------------------
* Cleanup CI (`#70 <https://github.com/ros/geometry_tutorials/issues/70>`_)
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
* update static_turtle_tf2_broadcaster.cpp to assign stamp value to now() (`#63 <https://github.com/ros/geometry_tutorials/issues/63>`_)
* Added compile and test Github action (`#60 <https://github.com/ros/geometry_tutorials/issues/60>`_)
* assign now variable to node now() value in frame broadcasters (`#61 <https://github.com/ros/geometry_tutorials/issues/61>`_)
* Udpate C and CPP standard (`#59 <https://github.com/ros/geometry_tutorials/issues/59>`_)
* Contributors: Alejandro Hernández Cordero, kenny_wang, kurshakuz

0.3.3 (2021-08-27)
------------------
* update listener node (`#53 <https://github.com/ros/geometry_tutorials/issues/53>`_)
* assign now value to the this->get_clock()->now() (`#54 <https://github.com/ros/geometry_tutorials/issues/54>`_)
* Fixed and dynamic frame broacaster nodes and launch files C++ (`#52 <https://github.com/ros/geometry_tutorials/issues/52>`_)
* update copyright year (`#51 <https://github.com/ros/geometry_tutorials/issues/51>`_)
* Update tags in the package.xml (`#50 <https://github.com/ros/geometry_tutorials/issues/50>`_)
* replace exec_depend to build_depend in package.xml (`#49 <https://github.com/ros/geometry_tutorials/issues/49>`_)
* Contributors: kurshakuz

0.3.2 (2021-08-09)
------------------
* turtle_tf2_cpp tutorial package (`#44 <https://github.com/ros/geometry_tutorials/issues/44>`_)
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
* Contributors: kurshakuz
