********************
SMACHA Documentation
********************

**SMACHA** is a meta-scripting, templating, and code generation engine for rapid
prototyping of `ROS <http://www.ros.org>`__ `SMACH
<http://wiki.ros.org/smach>`__ state machines.

`SMACH <http://wiki.ros.org/smach>`__ is an exceptionally useful and
comprehensive task-level architecture for state machine construction in
`ROS <http://wiki.ros.org/>`__-based robot control systems. However,
while it provides much in terms of power and flexibility, its overall
task-level simplicity can often be obfuscated at the script-level by
boilerplate code, intricate structure and lack of code reuse between
state machine prototypes.

SMACHA (short for `“State Machine Assembler”`, pronounced `“smasha”`) aims at
distilling the task-level simplicity of SMACH into compact `YAML <http://yaml.org/>`__
scripts in the foreground, while retaining all of its
power and flexibility in `Jinja2 <http://jinja.pocoo.org/docs/2.9/>`__-based
templates and a custom code generation engine in the background.
Thus SMACHA does not aim to replace SMACH, but to augment it.

Overview
========

Here is a high-level illustration of the main components of the SMACHA API:

.. figure:: _static/smacha_overview.png
   :alt: SMACHA API Overview

Here is a video demonstrating the use of SMACHA with the `Baxter robot simulator
<http://sdk.rethinkrobotics.com/wiki/Baxter_Simulator>`_ using the
`baxter_smacha <https://github.com/abr-ijs/baxter_smacha>`_ package:

.. youtube:: KRjY0bd4dLg

| 

Why SMACHA?
===========

SMACHA allows for arbitrarily complex SMACH state machine programs to be written
as much shorter YAML scripts that can be directly executed with ROS. Here is the
`"Nesting State Machines" example <http://wiki.ros.org/smach/Tutorials/Nesting%20State%20Machines>`_
from the `SMACH Tutorials <http://wiki.ros.org/smach/Tutorials>`_ described in a SMACHA
script:

.. literalinclude:: /../test/smacha_scripts/executive_smach_tutorials/state_machine_nesting2.yml
   :language: yaml

This demonstrates a reduction from 80 lines of raw SMACH Python code to 14 lines
of SMACHA script. Not bad.  However, if we employ
`YAML inline format <https://en.wikipedia.org/wiki/YAML#Syntax>`_ and
:doc:`SMACHA shorthand script syntax <Scripting/smacha_scripts>`, we can get this down to 5 lines:

.. _shorthand-example:

.. literalinclude:: /../test/smacha_scripts/executive_smach_tutorials/state_machine_nesting2_shorthand.yml
   :language: yaml

Outside of script brevity, SMACHA provides many other benefits and tools to
help rapidly prototype complex state machines for robot control.

Scripting
=========

.. toctree::
   :hidden:

   SMACHA Scripts<Scripting/smacha_scripts>
   Userdata<Scripting/userdata>
   Container States<Scripting/container_states>
   Sub-Scripts and Super-Scripts<Scripting/sub_scripts_and_super_scripts>
   Shorthand Script Syntax<Scripting/shorthand>

:doc:`Scripting/smacha_scripts` are parsed by the :doc:`Parser module
<API/smacha.parser>` and direct :doc:`the Generator module
<API/smacha.generator>` on how to combine :doc:`SMACHA templates
<Templating/smacha_templates>` using the :doc:`Templater module
<API/smacha.templater>`.

Some of the benefits of `meta-scripting <https://en.wikipedia.org/wiki/Metaprogramming>`__
in this way include:

* **quick, at-a-glance overviews** of state machine program intent,
* **easy script manipulation, reuse and restructuring** - SMACHA provides
  various utilities to automate some common script manipulation tasks, e.g.

  - the :ref:`Contain Tool <contain-tool>` for automatic containerization of
    state sequences,
  - the :ref:`Extract Tool <extract-tool>` for automatic conversion of commonly
    used container states to reusable sub-scripts,

* **streamlined ROS integration**, e.g.

  - loading of scripts onto the ROS parameter server and
  - executing them with service calls.


Templating
==========

.. toctree::
   :hidden:

   SMACHA Templates<Templating/smacha_templates>
   Template Anatomy<Templating/anatomy>
   Template Inheritance<Templating/inheritance>
   Template Metadata<Templating/metadata>
   Template Macros<Templating/macros>
   Core API Base Templates<Templating/core_api_base_templates>
   Core API Container Templates<Templating/core_api_container_templates>
   Core API State Templates<Templating/core_api_state_templates>
   Other Core API Templates<Templating/other_core_api_templates>

:doc:`Templating/smacha_templates` are filled out by :doc:`the Templater module
<API/smacha.templater>` as directed by :doc:`the Generator module
<API/smacha.generator>` and as determined by the structure of the :doc:`SMACHA
scripts <Scripting/smacha_scripts>` parsed by :doc:`the Parser module
<API/smacha.parser>`.

The use of templates comes with its own additional benefits:

* **intricate boilerplate code can be automatically filled**,
* **increased code reusability and modularity**, e.g.

  - common design patterns can be easily turned into state templates,
  - :doc:`template macros <Templating/macros>` make it easy to repeat common patterns between templates,
  - :doc:`template inheritance <Templating/inheritance>` helps avoid repetitive code blocks,
 
* **templates could be designed for use with frameworks other than SMACH and Python**

  - although this has not yet been tested, in theory templates could be written for use
    with other frameworks, e.g. `FlexBE <https://wiki.ros.org/flexbe>`_,
    `CoSTAR <https://github.com/cpaxton/costar_stack>`_, and others.
    

Code Generation
===============

.. toctree::
   :hidden:

   SMACHA Code Generator<CodeGeneration/smacha_code_generator>

The :doc:`CodeGeneration/smacha_code_generator` recursively processes :doc:`Scripting/smacha_scripts`
parsed by :doc:`API/smacha.parser`, manages the use of :doc:`Templating/smacha_templates` that
are filled out by :doc:`API/smacha.templater`, and renders the final result to
executable Python SMACH code.


Installation
============

Simply clone into the ``src`` directory of your catkin workspace,
update dependencies, and run
``catkin_make`` or ``catkin build`` from the root of the workspace.

.. code-block:: bash

   $ cd ~/catkin_ws/src
   $ git clone git@github.com:ReconCell/smacha.git
   $ cd ..
   $ rosdep install --from-paths src --ignore-src --rosdistro kinetic -y
   $ catkin build
   $ source ~/catkin_ws/devel/setup.bash

Usage
=====

Generate
--------

Given a SMACHA script (:download:`seq_nesting_1.yml
</../test/smacha_scripts/smacha_test_examples/seq_nesting_1.yml>`) and a set of templates, e.g. to replicate the code from
the `SMACH Nesting State Machines tutorial
<https://wiki.ros.org/smach/Tutorials/Nesting%20State%20Machines>`_, SMACH
Python code can be generated with the following command:

.. code-block:: bash

   $ rosrun smacha generate -t `rospack find smacha`/test/smacha_templates/executive_smach_tutorials -o state_machine_nesting2.py -v `rospack find smacha`/test/smacha_scripts/executive_smach_tutorials/state_machine_nesting2.yml

Here, the ``-t`` argument specifies custom template directories for this
particular tutorial, which may contain templates that override the core
templates.

The ``-o`` argument specifies a custom name for the generated output
file.

The ``-v`` argument tells SMACHA to print verbose processing output to
the terminal.

Further arguments and options may be explored by running
``rosrun smacha generate -h`` or ``rosrun smacha generate --help``.

Execute
-------

The script from the above example can also be directly executed provided that a `roscore <https://wiki.ros.org/roscore>`_ is running:

.. code-block:: bash

   $ rosrun smacha execute -t `rospack find smacha`/test/smacha_templates/executive_smach_tutorials -v `rospack find smacha`/test/smacha_scripts/executive_smach_tutorials/state_machine_nesting2.yml

Again, further arguments and options may be explored by running
``rosrun smacha execute -h`` or ``rosrun smacha execute --help``.

.. important:: A `roscore <https://wiki.ros.org/roscore>`_ must be running in order to execute scripts in this way!

Help
----

Help information may be printed for any template that contains
a :doc:`metadata block <Templating/metadata>` by running, e.g.
the following command for the :doc:`ServiceState template <API/Templates/ServiceState.tpl.py>`:

.. code-block:: bash

   $ rosrun smacha help ServiceState

Testing
=======

Tests may be performed by running:

.. code-block:: bash

   $ python `rospack find smacha`/test/executive_smach_tutorials_test.py
   $ python `rospack find smacha`/test/smacha_test_examples.py

Passing the ``-h`` argument as follows will bring up a list of some
other options.

Passing the ``-w`` argument will write the generated output files to
disk, which can subsequently be run as follows, e.g. for the
``state_machine_nesting2.py`` example:

.. code-block:: bash

   $ rosrun smacha state_machine_nesting2_generate_output.py

To run the original file script, use the following command:

.. code-block:: bash

   $ rosrun smacha state_machine_nesting2.py

**********
SMACHA API
**********

Modules
=======

.. toctree::
   :hidden:

   Parser<API/smacha.parser>
   Templater<API/smacha.templater>
   Generator<API/smacha.generator>
   Utilties<API/smacha.util>
   Exceptions<API/smacha.exceptions>

These are the main smacha package submodules:

- :doc:`Parser <API/smacha.parser>`: provides the :class:`smacha.parser.Parser` class for parsing :doc:`Scripting/smacha_scripts`.
- :doc:`Templater <API/smacha.templater>`: provides the :class:`smacha.templater.Templater` class as well as other functions for processing :doc:`Templating/smacha_templates`.
- :doc:`Generator <API/smacha.generator>`: provides the :class:`smacha.generator.Generator` class for generating executable Python SMACH code.
- :doc:`Util <API/smacha.util>`: provides various utilities.
- :doc:`Exceptions <API/smacha.exceptions>`: provides exception definitions.

Templates
=========

.. toctree::
   :hidden:

These are the core smacha package templates:

Base Templates
--------------

.. toctree::
   :hidden:

   Base<API/Templates/Base.tpl.py>

- :doc:`Base <API/Templates/Base.tpl.py>`: used for specifying the bare bones of a Python SMACH state machine script.

Container Templates
-------------------

.. toctree::
   :hidden:

   StateMachine<API/Templates/StateMachine.tpl.py>
   Concurrence<API/Templates/Concurrence.tpl.py>

- :doc:`StateMachine <API/Templates/StateMachine.tpl.py>`: used for inserting a `StateMachine container <http://wiki.ros.org/smach/Tutorials/StateMachine%20container>`__.
- :doc:`Concurrence <API/Templates/Concurrence.tpl.py>`: used for inserting a `Concurrence container <http://wiki.ros.org/smach/Tutorials/Concurrence%20container>`__.


State Templates
---------------

.. toctree::
   :hidden:

   CallbacksState<API/Templates/CallbacksState.tpl.py>
   ConditionalOutcomeState<API/Templates/ConditionalOutcomeState.tpl.py>
   DeleteFileState<API/Templates/DeleteFileState.tpl.py>
   PrintUserdataState<API/Templates/PrintUserdataState.tpl.py>
   PublishMsgState<API/Templates/PublishMsgState.tpl.py>
   RandomOutcomeState<API/Templates/RandomOutcomeState.tpl.py>
   ReadTopicState<API/Templates/ReadTopicState.tpl.py>
   ServiceState<API/Templates/ServiceState.tpl.py>
   SimpleActionState<API/Templates/SimpleActionState.tpl.py>
   SleepState<API/Templates/SleepState.tpl.py>
   TF2ListenerState<API/Templates/TF2ListenerState.tpl.py>
   TransformMsgState<API/Templates/TransformMsgState.tpl.py>
   WriteCSVFileState<API/Templates/WriteCSVFileState.tpl.py>

- :doc:`CallbacksState <API/Templates/CallbacksState.tpl.py>`: used for creating lambda function callbacks.
- :doc:`ConditionalOutcomeState <API/Templates/ConditionalOutcomeState.tpl.py>`: used for conditionally selecting an outcome.
- :doc:`DeleteFileState <API/Templates/DeleteFileState.tpl.py>`: used for deleting files from the file system.
- :doc:`PrintUserdataState <API/Templates/PrintUserdataState.tpl.py>`: used for printing userdata entries to standard output.
- :doc:`PublishMsgState <API/Templates/PublishMsgState.tpl.py>`: used for publishing userdata ROS message entries to topics.
- :doc:`RandomOutcomeState <API/Templates/RandomOutcomeState.tpl.py>`: used for selecting a random outcome from a specified list of outcomes.
- :doc:`ReadTopicState <API/Templates/ReadTopicState.tpl.py>`: used for reading ROS messages from topics to userdata.
- :doc:`ServiceState <API/Templates/ServiceState.tpl.py>`: used for inserting a `ServiceState <http://wiki.ros.org/smach/Tutorials/ServiceState>`_.
- :doc:`SimpleActionState <API/Templates/SimpleActionState.tpl.py>`: used for inserting a `SimpleActionState <http://wiki.ros.org/smach/Tutorials/SimpleActionState>`_.
- :doc:`TF2ListenerState <API/Templates/TF2ListenerState.tpl.py>`: used for reading `TF2 <https://wiki.ros.org/tf2>`_ transforms.
- :doc:`TransformMsgState <API/Templates/TransformMsgState.tpl.py>`: used for transforming userdata ROS messages from one `tf <https://wiki.ros.org/tf>`_ frame to another.
- :doc:`WriteCSVFileState <API/Templates/WriteCSVFileState.tpl.py>`: used for writing `CSV files <https://en.wikipedia.org/wiki/Comma-separated_values>`_ to the file system.

Other Templates
---------------

.. toctree::
   :hidden:

   MsgPublisherObserver<API/Templates/MsgPublisherObserver.tpl.py>
   ParseJointTrajectoryPoint<API/Templates/ParseJointTrajectoryPoint.tpl.py>
   ParsePointCloud<API/Templates/ParsePointCloud.tpl.py>
   ParsePointCloud2<API/Templates/ParsePointCloud2.tpl.py>
   ParsePointStamped<API/Templates/ParsePointStamped.tpl.py>
   ParsePose<API/Templates/ParsePose.tpl.py>
   ParsePoseArray<API/Templates/ParsePoseArray.tpl.py>
   ParsePoseStamped<API/Templates/ParsePoseStamped.tpl.py>
   ParseTransformStamped<API/Templates/ParseTransformStamped.tpl.py>
   State<API/Templates/State.tpl.py>
   TF2ListenerSingleton<API/Templates/TF2ListenerSingleton.tpl.py>
   Utilities<API/Templates/Utils.tpl.py>
   WaitForMsgState<API/Templates/WaitForMsgState.tpl.py>

- :doc:`MsgPublisherObserver <API/Templates/MsgPublisherObserver.tpl.py>`: provides a helper class for the :doc:`PublishMsgState <API/Templates/PublishMsgState.tpl.py>` template.
- :doc:`ParseJointTrajectoryPoint <API/Templates/ParseJointTrajectoryPoint.tpl.py>`: provides a helper function for parsing `trajectory_msgs/JointTrajectoryPoint <https://docs.ros.org/api/trajectory_msgs/html/msg/JointTrajectoryPoint.html>`_ types.
- :doc:`ParsePointCloud <API/Templates/ParsePointCloud.tpl.py>`: provides a helper function for parsing `sensor_msgs/PointCloud <https://docs.ros.org/api/sensor_msgs/html/msg/PointCloud.html>`_ types.
- :doc:`ParsePointCloud2 <API/Templates/ParsePointCloud2.tpl.py>`: provides a helper function for parsing `sensor_msgs/PointCloud2 <https://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html>`_ types.
- :doc:`ParsePointStamped <API/Templates/ParsePointStamped.tpl.py>`: provides a helper function for parsing `geometry_msgs/PointStamped <https://docs.ros.org/api/geometry_msgs/html/msg/PointStamped.html>`_ types.
- :doc:`ParsePose <API/Templates/ParsePose.tpl.py>`: provides a helper function for parsing `geometry_msgs/Pose <https://docs.ros.org/api/geometry_msgs/html/msg/Pose.html>`_ types.
- :doc:`ParsePoseArray <API/Templates/ParsePoseArray.tpl.py>`: provides a helper function for parsing `geometry_msgs/PoseArray <https://docs.ros.org/api/geometry_msgs/html/msg/PoseArray.html>`_ types.
- :doc:`ParsePoseStamped <API/Templates/ParsePoseStamped.tpl.py>`: provides a helper function for parsing `geometry_msgs/PoseStamped <https://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html>`_ types.
- :doc:`ParseTransformStamped <API/Templates/ParseTransformStamped.tpl.py>`: provides a helper function for parsing `geometry_msgs/TransformStamped <https://docs.ros.org/api/geometry_msgs/html/msg/TransformStamped.html>`_ types.
- :doc:`State <API/Templates/State.tpl.py>`: contains code common to all state templates.
- :doc:`TF2ListenerSingleton <API/Templates/TF2ListenerSingleton.tpl.py>`: provides a helper class for the :doc:`TF2ListenerState <API/Templates/TF2ListenerState.tpl.py>` template.
- :doc:`Utils <API/Templates/Utils.tpl.py>`: contains template macros and other utilities.
- :doc:`WaitForMsgState <API/Templates/WaitForMsgState.tpl.py>`: provides a helper class for the :doc:`ReadTopicState <API/Templates/ReadTopicState.tpl.py>` template.

*********************
Other SMACHA Packages
*********************

SMACHA ROS
==========

The `SMACHA ROS package <https://reconcell.gitlab.io/smacha/smacha_ros/smacha_ros.html>`_ contains a SMACHA server
that provides ROS services, ROS parameter server integration, and other functions.

SMACHA GUI
==========

The `SMACHA GUI package <https://reconcell.gitlab.io/reconcell_docs/UserManuals/smacha_gui/index.html>`_ provides a
visual programming interface for SMACHA for use within the `ReconCell Project <https://reconcell.gitlab.io/reconcell_docs/#>`_.

********************
References/Citations
********************

If you use SMACHA in your work, please consider citing the following paper:

B. Ridge, T. Gašpar, and A. Ude. `Rapid State Machine Assembly for Modular Robot
Control using Meta-Scripting, Templating and Code Generation
<https://ieeexplore.ieee.org/abstract/document/8246943>`_. *In IEEE-RAS 17th
International Conference on Humanoid Robots (Humanoids)*, pages 661–668,
Birmingham, UK, November 2017.

******************
Indices and tables
******************

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
