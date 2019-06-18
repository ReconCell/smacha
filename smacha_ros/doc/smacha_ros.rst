************************
SMACHA ROS Documentation
************************

**SMACHA ROS** is a package that provides ROS integration for `SMACHA <https://reconcell.gitlab.io/smacha/smacha/smacha.html>`_.

Its main component is a SMACHA server that allows for templates to be
loaded from disparate packages and accessed from anywhere within the ROS
network, loads template metadata onto the parameter server for use with,
e.g. the `ReconCell SMACHA GUI <https://reconcell.gitlab.io/reconcell_docs/UserManuals/smacha_gui/index.html>`_,
and provides services for each of the main SMACHA functions.

SMACHA Server
=============

Services
--------

The SMACHA server provides a number of `ROS services <https://wiki.ros.org/Services>`_ where,
in each case, service requests may be handled in one of the following three ways:

* the request itself is a SMACHA script string,
* the request refers to a `ROS parameter`__ containing a SMACHA script string,
* the request refers to a `ROS parameter`__ containing a SMACHA script filename.

__ https://wiki.ros.org/Parameter%20Server
__ https://wiki.ros.org/Parameter%20Server

The currently available services are as follows:

* the ``Generate`` service: uses the `SMACHA Generator`__ to generate SMACH Python code from SMACHA scripts,
* the ``Contain`` service: uses the `SMACHA Contain tool`__ to containerize linear state sequences in SMACHA scripts,
* the ``Extract`` service: uses the `SMACHA Extract tool`__ to extract sub-scripts from super-scripts.

__ https://reconcell.gitlab.io/smacha/smacha/CodeGeneration/smacha_code_generator.html
__ https://reconcell.gitlab.io/smacha/smacha/Scripting/container_states.html#the-contain-tool
__ https://reconcell.gitlab.io/smacha/smacha/Scripting/sub_scripts_and_super_scripts.html#the-extract-tool

Template Metadata Parameters
----------------------------

Template metadata is automatically published to the ROS parameter server for
every template loaded by the parameter server (assuming a ``meta`` block
is present in the template).

Installation
============

The ``smacha_ros`` package forms a part of the main `SMACHA stack
<https://github.com/ReconCell/smacha>`_, so it is installed alongside SMACHA by
following its `installation instructions
<https://reconcell.gitlab.io/smacha/smacha/smacha.html#installation>`_.

Usage
=====

Launching a SMACHA Server
-------------------------

Launching a SMACHA server can be done as follows:

.. code-block:: bash

   $ rosrun smacha_ros smacha_server -t `rospack find smacha`/test/smacha_templates/smacha_test_examples

Here, the ``-t`` argument specifies directories containing additional templates that should be loaded by
the server outside of the `core API templates <https://reconcell.gitlab.io/smacha/smacha/Templating/core_api_templates.html>`_
which are always loaded by default.

Further arguments and options may be explored by running
``rosrun smacha_ros smacha_server -h`` or ``rosrun smacha_ros smacha_server --help``.

Using Scripts with the ROS Parameter Server
-------------------------------------------

Scripts may be used with the ROS parameter server in one of two ways, by either
setting a parameter to hold the script filename:

.. code-block:: bash

   $ rosparam set /seq_filename `rospack find smacha`/test/smacha_scripts/smacha_test_examples/seq.yml

or by loading the script itself (i.e. the contents of the file) onto a parameter:

.. code-block:: bash

   $ rosparam load `rospack find smacha`/test/smacha_scripts/smacha_test_examples/seq.yml /seq_script

Since SMACHA scripts are written in YAML and the ROS parameter server is designed to handle YAML,
this latter method is advantageous in that once the script has been loaded, different parts of the script,
can be directly accessed, e.g.

.. code-block:: bash

   $ rosparam get /seq_script/template

returns the name of the `base template <https://reconcell.gitlab.io/smacha/smacha/Templating/smacha_templates.html#base-templates>`_ being used by the script:

.. code-block:: bash

   $ rosparam get /seq_script/template
   $ Base

Calling the Generate Service
----------------------------

Calling the ``Generate`` service using the parameter containing the filename from the above example:

.. code-block:: bash

   rosservice call /smacha/generate /seq_filename

of using the parameter containing the script itself:

.. code-block:: bash

   rosservice call /smacha/generate /seq_script

will result in the same outcome, that is, the generated SMACH Python code string being returned.

Calling the Contain Service
---------------------------

The following command demonstrates how to call the contain service in order to replicate
the example from the `SMACHA Container States tutorial <https://reconcell.gitlab.io/smacha/smacha/Scripting/container_states.html>`_:

.. code-block:: bash

   $ rosservice call /smacha/contain /seq_filename SUB StateMachine [FOO_0,FOO_1]

Calling the Extract Service
---------------------------

If we assign the output of the previous ``Contain`` service call to an environment variable:

.. code-block:: bash

   $ CONTAINED_SEQ=`rosservice call /smacha/contain /seq_filename SUB StateMachine [FOO_0,FOO_1]`

then we can load the generated script onto the parameter server:

.. code-block:: bash

   $ rosparam set /contained_seq "$CONTAINED_SEQ"

before calling the ``Extract`` service in order to extract the ``SUB`` state from the script:

.. code-block:: bash

   $ rosservice call /smacha/extract /contained_seq/script SUB

Templating
==========

.. toctree::
   :hidden:

   ROS State Templates<Templating/core_api_state_templates>
   Other ROS Templates<Templating/other_core_api_templates>

The SMACHA ROS package provides :doc:`ROS state templates <Templating/core_api_state_templates>` as well as :doc:`other ROS templates<Templating/other_core_api_templates>`
in order to provide ROS integration and additional functionality, for example:

* **Templates for publishing arbitrary messages to topics**,
* **Templates for performing TF transforms**,
* **Templates for parsing messages into selected types**,
* **Templates for recording ROS bags**,
* etc.

*********************
Other SMACHA Packages
*********************

SMACHA
======

The `SMACHA package <https://reconcell.gitlab.io/smacha/smacha/smacha.html>`_ contains all of the
main SMACHA functionality.

SMACHA GUI
==========

The `SMACHA GUI package <https://reconcell.gitlab.io/reconcell_docs/DevManuals/smacha_gui/index.html>`_ provides a
visual programming interface for SMACHA for use within the `ReconCell Project <https://reconcell.gitlab.io/reconcell_docs/#>`_.

**************
SMACHA ROS API
**************

Templates
=========

.. toctree::
   :hidden:

These are the core smacha_ros package templates:

State Templates
---------------

.. toctree::
   :hidden:

   PublishMsgState<API/Templates/PublishMsgState.tpl.py>
   PublishObserverMsgState<API/Templates/PublishObserverMsgState.tpl.py>
   ReadTopicState<API/Templates/ReadTopicState.tpl.py>
   RecordROSBagState<API/Templates/RecordROSBagState.tpl.py>
   ServiceState<API/Templates/ServiceState.tpl.py>
   SimpleActionState<API/Templates/SimpleActionState.tpl.py>
   SleepState<API/Templates/SleepState.tpl.py>
   TF2ListenerState<API/Templates/TF2ListenerState.tpl.py>
   TransformMsgState<API/Templates/TransformMsgState.tpl.py>


- :doc:`PublishMsgState <API/Templates/PublishMsgState.tpl.py>`: used for publishing userdata ROS message entries to topics.
- :doc:`PublishObserverMsgState <API/Templates/PublishObserverMsgState.tpl.py>`: used for publishing userdata ROS message entries to topics using an `observer pattern <https://en.wikipedia.org/wiki/Observer_pattern>`_.
- :doc:`ReadTopicState <API/Templates/ReadTopicState.tpl.py>`: used for reading ROS messages from topics to userdata.
- :doc:`RecordROSBagState <API/Templates/RecordROSBagState.tpl.py>`: used for recording topic data to `ROS bags <https://wiki.ros.org/Bags>`_.
- :doc:`ServiceState <API/Templates/ServiceState.tpl.py>`: used for inserting a `ServiceState <http://wiki.ros.org/smach/Tutorials/ServiceState>`_.
- :doc:`SimpleActionState <API/Templates/SimpleActionState.tpl.py>`: used for inserting a `SimpleActionState <http://wiki.ros.org/smach/Tutorials/SimpleActionState>`_.
- :doc:`TF2ListenerState <API/Templates/TF2ListenerState.tpl.py>`: used for reading `TF2 <https://wiki.ros.org/tf2>`_ transforms.
- :doc:`TransformMsgState <API/Templates/TransformMsgState.tpl.py>`: used for transforming userdata ROS messages from one `tf <https://wiki.ros.org/tf>`_ frame to another.


Other Templates
---------------

.. toctree::
   :hidden:
   
   MsgPublisher<API/Templates/MsgPublisher.tpl.py>
   MsgPublisherObserver<API/Templates/MsgPublisherObserver.tpl.py>
   ParseJointTrajectoryPoint<API/Templates/ParseJointTrajectoryPoint.tpl.py>
   ParsePointCloud<API/Templates/ParsePointCloud.tpl.py>
   ParsePointCloud2<API/Templates/ParsePointCloud2.tpl.py>
   ParsePointStamped<API/Templates/ParsePointStamped.tpl.py>
   ParsePose<API/Templates/ParsePose.tpl.py>
   ParsePoseArray<API/Templates/ParsePoseArray.tpl.py>
   ParsePoseStamped<API/Templates/ParsePoseStamped.tpl.py>
   ParseTransformStamped<API/Templates/ParseTransformStamped.tpl.py>
   ROSBagAPIThreadRecorder<API/Templates/ROSBagAPIThreadRecorder.tpl.py>
   ROSBagCLIProcessRecorder<API/Templates/ROSBagCLIProcessRecorder.tpl.py>
   TF2ListenerSingleton<API/Templates/TF2ListenerSingleton.tpl.py>
   WaitForMsgState<API/Templates/WaitForMsgState.tpl.py>

- :doc:`MsgPublisher <API/Templates/MsgPublisher.tpl.py>`: provides a helper class for the :doc:`PublishMsgState <API/Templates/PublishMsgState.tpl.py>` template.
- :doc:`MsgPublisherObserver <API/Templates/MsgPublisherObserver.tpl.py>`: provides a helper class for the :doc:`PublishObserverMsgState <API/Templates/PublishObserverMsgState.tpl.py>` template.
- :doc:`ParseJointTrajectoryPoint <API/Templates/ParseJointTrajectoryPoint.tpl.py>`: provides a helper function for parsing `trajectory_msgs/JointTrajectoryPoint <https://docs.ros.org/api/trajectory_msgs/html/msg/JointTrajectoryPoint.html>`_ types.
- :doc:`ParsePointCloud <API/Templates/ParsePointCloud.tpl.py>`: provides a helper function for parsing `sensor_msgs/PointCloud <https://docs.ros.org/api/sensor_msgs/html/msg/PointCloud.html>`_ types.
- :doc:`ParsePointCloud2 <API/Templates/ParsePointCloud2.tpl.py>`: provides a helper function for parsing `sensor_msgs/PointCloud2 <https://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html>`_ types.
- :doc:`ParsePointStamped <API/Templates/ParsePointStamped.tpl.py>`: provides a helper function for parsing `geometry_msgs/PointStamped <https://docs.ros.org/api/geometry_msgs/html/msg/PointStamped.html>`_ types.
- :doc:`ParsePose <API/Templates/ParsePose.tpl.py>`: provides a helper function for parsing `geometry_msgs/Pose <https://docs.ros.org/api/geometry_msgs/html/msg/Pose.html>`_ types.
- :doc:`ParsePoseArray <API/Templates/ParsePoseArray.tpl.py>`: provides a helper function for parsing `geometry_msgs/PoseArray <https://docs.ros.org/api/geometry_msgs/html/msg/PoseArray.html>`_ types.
- :doc:`ParsePoseStamped <API/Templates/ParsePoseStamped.tpl.py>`: provides a helper function for parsing `geometry_msgs/PoseStamped <https://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html>`_ types.
- :doc:`ParseTransformStamped <API/Templates/ParseTransformStamped.tpl.py>`: provides a helper function for parsing `geometry_msgs/TransformStamped <https://docs.ros.org/api/geometry_msgs/html/msg/TransformStamped.html>`_ types.
- :doc:`ROSBagAPIThreadRecorder <API/Templates/ROSBagAPIThreadRecorder.tpl.py>`: provides a helper class for the :doc:`RecordROSBagState <API/Templates/RecordROSBagState.tpl.py>` template that allows for ROS bags to be recorded using the `ROS bag Python application programming interface <https://wiki.ros.org/rosbag/Code%20API#Python_API>`_.
- :doc:`ROSBagCLIProcessRecorder <API/Templates/ROSBagCLIProcessRecorder.tpl.py>`: provides a helper class for the :doc:`RecordROSBagState <API/Templates/RecordROSBagState.tpl.py>` template that allows for ROS bags to be recorded using the `ROS bag command line interface <https://wiki.ros.org/rosbag/Commandline>`_.
- :doc:`TF2ListenerSingleton <API/Templates/TF2ListenerSingleton.tpl.py>`: provides a helper class for the :doc:`TF2ListenerState <API/Templates/TF2ListenerState.tpl.py>` template.
- :doc:`WaitForMsgState <API/Templates/WaitForMsgState.tpl.py>`: provides a helper class for the :doc:`ReadTopicState <API/Templates/ReadTopicState.tpl.py>` template.


******************
Indices and tables
******************

* :ref:`genindex`
* :ref:`search`
