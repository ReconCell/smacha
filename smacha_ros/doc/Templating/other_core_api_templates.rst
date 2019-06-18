************************
Other Core API Templates
************************

MsgPublisher
====================

The :doc:`MsgPublisher <../API/Templates/MsgPublisher.tpl.py>` template provides a helper class for :doc:`../API/Templates/PublishMsgState.tpl.py`  and is specified as follows:

.. program-output:: rosrun smacha help -t ../src/smacha_ros/templates -n MsgPublisher

MsgPublisherObserver
====================

The :doc:`MsgPublisherObserver <../API/Templates/MsgPublisherObserver.tpl.py>` template provides a helper class for :doc:`../API/Templates/PublishObserverMsgState.tpl.py`  and is specified as follows:

.. program-output:: rosrun smacha help -t ../src/smacha_ros/templates -n MsgPublisherObserver

ParseJointTrajectoryPoint
=========================

The :doc:`ParseJointTrajectoryPoint <../API/Templates/ParseJointTrajectoryPoint.tpl.py>` template provides a helper function for parsing `trajectory_msgs/JointTrajectoryPoint <https://docs.ros.org/api/trajectory_msgs/html/msg/JointTrajectoryPoint.html>`_ types and is specified as follows:

.. program-output:: rosrun smacha help -t ../src/smacha_ros/templates -n ParseJointTrajectoryPoint

ParsePointCloud
===============

The :doc:`ParsePointCloud <../API/Templates/ParsePointCloud.tpl.py>` template provides a helper function for parsing `sensor_msgs/PointCloud <https://docs.ros.org/api/sensor_msgs/html/msg/PointCloud.html>`_ types and is specified as follows:

.. program-output:: rosrun smacha help -t ../src/smacha_ros/templates -n ParsePointCloud

ParsePointCloud2
================

The :doc:`ParsePointCloud2 <../API/Templates/ParsePointCloud2.tpl.py>` template provides a helper function for parsing `sensor_msgs/PointCloud2 <https://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html>`_ types and is specified as follows:

.. program-output:: rosrun smacha help -t ../src/smacha_ros/templates -n ParsePointCloud2

ParsePointStamped
=================

The :doc:`ParsePointStamped <../API/Templates/ParsePointStamped.tpl.py>` template provides a helper function for parsing `geometry_msgs/PointStamped <https://docs.ros.org/api/geometry_msgs/html/msg/PointStamped.html>`_ types and is specified as follows:

.. program-output:: rosrun smacha help -t ../src/smacha_ros/templates -n ParsePointStamped

ParsePose 
=========

The :doc:`ParsePose <../API/Templates/ParsePose.tpl.py>` template provides a helper function for parsing `geometry_msgs/Pose <https://docs.ros.org/api/geometry_msgs/html/msg/Pose.html>`_ types and is specified as follows:

.. program-output:: rosrun smacha help -t ../src/smacha_ros/templates -n ParsePose

ParsePoseArray
==============

The :doc:`ParsePoseArray <../API/Templates/ParsePoseArray.tpl.py>` template provides a helper function for parsing `geometry_msgs/PoseArray <https://docs.ros.org/api/geometry_msgs/html/msg/PoseArray.html>`_ types and is specified as follows:

.. program-output:: rosrun smacha help -t ../src/smacha_ros/templates -n ParsePoseArray

ParsePoseStamped
================

The :doc:`ParsePoseStamped <../API/Templates/ParsePoseStamped.tpl.py>` template provides a helper function for parsing `geometry_msgs/PoseStamped <https://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html>`_ types and is specified as follows:

.. program-output:: rosrun smacha help -t ../src/smacha_ros/templates -n ParsePoseStamped

ParseTransformStamped
=====================

The :doc:`ParseTransformStamped <../API/Templates/ParseTransformStamped.tpl.py>` provides a helper function for parsing `geometry_msgs/TransformStamped <https://docs.ros.org/api/geometry_msgs/html/msg/TransformStamped.html>`_ types and is specified as follows:

.. program-output:: rosrun smacha help -t ../src/smacha_ros/templates -n ParseTransformStamped

ROSBagAPIThreadRecorder
=======================

The :doc:`ROSBagAPIThreadRecorder <../API/Templates/ROSBagAPIThreadRecorder.tpl.py>` provides a helper class for the :doc:`RecordROSBagState <../API/Templates/RecordROSBagState.tpl.py>` template that allows for ROS bags to be recorded using the `ROS bag Python application programming interface <https://wiki.ros.org/rosbag/Code%20API#Python_API>`_ and is specified as follows:

.. program-output:: rosrun smacha help -t ../src/smacha_ros/templates -n ROSBagAPIThreadRecorder

ROSBagCLIProcessRecorder
========================

The :doc:`ROSBagCLIProcessRecorder <../API/Templates/ROSBagCLIProcessRecorder.tpl.py>` provides a helper class for the :doc:`RecordROSBagState <../API/Templates/RecordROSBagState.tpl.py>` template that allows for ROS bags to be recorded using the `ROS bag command line interface <https://wiki.ros.org/rosbag/Commandline>`_ and is specified as follows:

.. program-output:: rosrun smacha help -t ../src/smacha_ros/templates -n ROSBagCLIProcessRecorder

WaitForMsgState
===============

The :doc:`WaitForMsgState <../API/Templates/WaitForMsgState.tpl.py>` template provides a helper class for :doc:`../API/Templates/ReadTopicState.tpl.py` and is specified as follows:

.. program-output:: rosrun smacha help -t ../src/smacha_ros/templates -n WaitForMsgState
