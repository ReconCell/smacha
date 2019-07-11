************************
Core API State Templates
************************

PublishMsgState
===============

The :doc:`PublishMsgState <../API/Templates/PublishMsgState.tpl.py>` template is used for publishing userdata ROS message entries to topics and is specified as follows:

.. program-output:: rosrun smacha help -t ../src/smacha_ros/templates -n PublishMsgState

PublishObserverMsgState
=======================

The :doc:`PublishObserverMsgState <../API/Templates/PublishObserverMsgState.tpl.py>` template is used for publishing userdata ROS message entries to topics using an `observer pattern <https://en.wikipedia.org/wiki/Observer_pattern>`_ and is specified as follows:

.. program-output:: rosrun smacha help -t ../src/smacha_ros/templates -n PublishObserverMsgState

ReadTopicState
==============

The :doc:`ReadTopicState <../API/Templates/ReadTopicState.tpl.py>` template is used for reading ROS messages from topics and is specified as follows:

.. program-output:: rosrun smacha help -t ../src/smacha_ros/templates -n ReadTopicState

RecordROSBagState
=================

The :doc:`RecordROSBagState <../API/Templates/RecordROSBagState.tpl.py>` template is used for recording topic data to `ROS bags <https://wiki.ros.org/Bags>`_ and is specified as follows:

.. program-output:: rosrun smacha help -t ../src/smacha_ros/templates -n RecordROSBagState

ServiceState
============

The :doc:`ServiceState <../API/Templates/ServiceState.tpl.py>` template is used for inserting a `ServiceState <http://wiki.ros.org/smach/Tutorials/ServiceState>`__ and is specified as follows:

.. program-output:: rosrun smacha help -t ../src/smacha_ros/templates -n ServiceState

SimpleActionState
=================

The :doc:`SimpleActionState <../API/Templates/SimpleActionState.tpl.py>` template is used for inserting a `SimpleActionState <http://wiki.ros.org/smach/Tutorials/SimpleActionState>`__ and is specified as follows:

.. program-output:: rosrun smacha help -t ../src/smacha_ros/templates -n SimpleActionState

TF2ListenerState
================

The :doc:`TF2ListenerState <../API/Templates/TF2ListenerState.tpl.py>` template is used for reading TF2 transforms and is specified as follows:

.. program-output:: rosrun smacha help -t ../src/smacha_ros/templates -n TF2ListenerState

TransformMsgState
=================

The :doc:`TransformMsgState <../API/Templates/TransformMsgState.tpl.py>` template is used for transforming userdata ROS messages from one `tf <https://wiki.ros.org/tf>`_ frame to another and is specified as follows:

.. program-output:: rosrun smacha help -t ../src/smacha_ros/templates -n TransformMsgState
