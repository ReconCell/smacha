************************
Core API State Templates
************************

PublishMsgState
===============

The :doc:`PublishMsgState <../API/Templates/PublishMsgState.tpl.py>` template is used for publishing userdata ROS message entries to topics and is specified as follows:

.. program-output:: ./execute_sourced.sh rosrun smacha help PublishMsgState -n -t ../src/smacha_ros/templates
    :shell

ReadTopicState
==============

The :doc:`ReadTopicState <../API/Templates/ReadTopicState.tpl.py>` template is used for reading ROS messages from topics and is specified as follows:

.. program-output:: $(rospack find smacha)/scripts/help ReadTopicState -n -t $(rospack find smacha_ros)/src/smacha_ros/templates
  :shell:

ServiceState
============

The :doc:`ServiceState <../API/Templates/ServiceState.tpl.py>` template is used for inserting a `ServiceState <http://wiki.ros.org/smach/Tutorials/ServiceState>`__ and is specified as follows:

.. program-output:: ../../smacha/scripts/help ServiceState -n -t ../src/smacha_ros/templates

SimpleActionState
=================

The :doc:`SimpleActionState <../API/Templates/SimpleActionState.tpl.py>` template is used for inserting a `SimpleActionState <http://wiki.ros.org/smach/Tutorials/SimpleActionState>`__ and is specified as follows:

.. program-output:: ../../smacha/scripts/help SimpleActionState -n -t ../src/smacha_ros/templates

TF2ListenerState
================

The :doc:`TF2ListenerState <../API/Templates/TF2ListenerState.tpl.py>` template is used for reading TF2 transforms and is specified as follows:

.. program-output:: ../../smacha/scripts/help TF2ListenerState -n -t ../src/smacha_ros/templates

TransformMsgState
=================

The :doc:`TransformMsgState <../API/Templates/TransformMsgState.tpl.py>` template is used for transforming userdata ROS messages from one `tf <https://wiki.ros.org/tf>`_ frame to another and is specified as follows:

.. program-output:: ../../smacha/scripts/help TransformMsgState -n -t ../src/smacha_ros/templates
