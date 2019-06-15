************************
Core API State Templates
************************

CallbacksState
==============

The :doc:`CallbacksState <../API/Templates/CallbacksState.tpl.py>` template is used for creating lambda function callbacks and is specified as follows:

.. program-output:: ../scripts/help CallbacksState -n

ConditionalOutcomeState
=======================

The :doc:`ConditionalOutcomeState <../API/Templates/ConditionalOutcomeState.tpl.py>` template is used for conditionally selecting an outcome and is specified as follows:

.. program-output:: ../scripts/help ConditionalOutcomeState -n

DeleteFileState
===============

The :doc:`DeleteFileState <../API/Templates/DeleteFileState.tpl.py>` template is used deleting files from the file system and is specified as follows:

.. program-output:: ../scripts/help DeleteFileState -n

PrintUserdataState
==================

The :doc:`PrintUserdataState <../API/Templates/PrintUserdataState.tpl.py>` template is used for printing userdata entries to standard output and is specified as follows:

.. program-output:: ../scripts/help PrintUserdataState -n

PublishMsgState
===============

The :doc:`PublishMsgState <../API/Templates/PublishMsgState.tpl.py>` template is used for publishing userdata ROS message entries to topics and is specified as follows:

.. program-output:: ../scripts/help PublishMsgState -n

PublishObserverMsgState
=======================

The :doc:`PublishObserverMsgState <../API/Templates/PublishObserverMsgState.tpl.py>` template is used for publishing userdata ROS message entries to topics using an `observer pattern <https://en.wikipedia.org/wiki/Observer_pattern>`_ and is specified as follows:

.. program-output:: ../scripts/help PublishObserverMsgState -n

RandomOutcomeState
==================

The :doc:`RandomOutcomeState <../API/Templates/RandomOutcomeState.tpl.py>` template is used for selecting a random outcome from a specified list of outcomes and is specified as follows:

.. program-output:: ../scripts/help RandomOutcomeState -n

ReadTopicState
==============

The :doc:`ReadTopicState <../API/Templates/ReadTopicState.tpl.py>` template is used for reading ROS messages from topics and is specified as follows:

.. program-output:: ../scripts/help ReadTopicState -n

RecordROSBagState
=================

The :doc:`RecordROSBagState <../API/Templates/RecordROSBagState.tpl.py>` template is used for recording topic data to `ROS bags <https://wiki.ros.org/Bags>`_ and is specified as follows:

.. program-output:: ../scripts/help RecordROSBagState -n

ServiceState
============

The :doc:`ServiceState <../API/Templates/ServiceState.tpl.py>` template is used for inserting a `ServiceState <http://wiki.ros.org/smach/Tutorials/ServiceState>`__ and is specified as follows:

.. program-output:: ../scripts/help ServiceState -n

SimpleActionState
=================

The :doc:`SimpleActionState <../API/Templates/SimpleActionState.tpl.py>` template is used for inserting a `SimpleActionState <http://wiki.ros.org/smach/Tutorials/SimpleActionState>`__ and is specified as follows:

.. program-output:: ../scripts/help SimpleActionState -n

TF2ListenerState
================

The :doc:`TF2ListenerState <../API/Templates/TF2ListenerState.tpl.py>` template is used for reading TF2 transforms and is specified as follows:

.. program-output:: ../scripts/help TF2ListenerState -n

TransformMsgState
=================

The :doc:`TransformMsgState <../API/Templates/TransformMsgState.tpl.py>` template is used for transforming userdata ROS messages from one `tf <https://wiki.ros.org/tf>`_ frame to another and is specified as follows:

.. program-output:: ../scripts/help TransformMsgState -n

WriteCSVFileState
=================

The :doc:`WriteCSVFileState <../API/Templates/WriteCSVFileState.tpl.py>` template is used for writing `CSV files <https://en.wikipedia.org/wiki/Comma-separated_values>`_ to the file system and is specified as follows:

.. program-output:: ../scripts/help WriteCSVFileState -n
