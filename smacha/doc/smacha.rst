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

.. figure:: _static/smacha_overview.png
   :alt: SMACHA API Overview

Why SMACHA?
===========

SMACHA allows for arbitrarily complex SMACH state machine programs to be written
as compact YAML scripts that can be directly executed with ROS. Here is the
`"Nesting State Machines" example <http://wiki.ros.org/smach/Tutorials/Nesting%20State%20Machines>`__
from the `SMACH Tutorials <http://wiki.ros.org/smach/Tutorials>`__ described in a SMACHA
script:

.. literalinclude:: /../test/smacha_scripts/executive_smach_tutorials/state_machine_nesting2.yml
   :language: yaml

This demonstrates a reduction from 80 lines of raw SMACH Python code to 15 lines
of SMACHA script. Not bad.
SMACHA provides many other benefits and tools to
help rapidly prototype complex state machines for robot control.

Scripting
=========

.. toctree::
   :hidden:

   SMACHA Scripts<Scripting/smacha_scripts>
   Userdata<Scripting/userdata>
   Container States<Scripting/container_states>
   Sub-Scripts and Super-Scripts<Scripting/sub_scripts_and_super_scripts>

:doc:`Scripting/smacha_scripts` are `YAML <http://yaml.org/>`__ files that describe
how SMACHA should generate SMACH code. They are parsed by :doc:`API/smacha.parser` and direct
:doc:`API/smacha.generator` on how to combine :doc:`Templating/smacha_templates` using the
:doc:`API/smacha.templater`.

Some of the benefits of `meta-scripting <https://en.wikipedia.org/wiki/Metaprogramming>`__
in this way include:

* **quick, at-a-glance overviews** of state machine program intent,
* **easy script manipulation, reuse and restructuring**,
* **streamlined ROS integration**, e.g. loading of scripts onto the parameter server.

SMACHA provides various utilities to automate some common state machine script manipulation tasks,
such as the :ref:`Contain Tool <contain-tool>` for
**automatic containerization of state sequences**, and
the :ref:`Extract Tool <extract-tool>` for
**automatic conversion of commonly used container states to reusable sub-scripts**.

Templating
==========

.. toctree::
   :hidden:

   SMACHA Templates<Templating/smacha_templates>
   Anatomy<Templating/anatomy>
   Inheritance<Templating/inheritance>
   Metadata<Templating/metadata>
   Core API Templates<Templating/core_api_templates>

:doc:`Templating/smacha_templates` are Jinja2-based Python SMACH code templates
containing variables that are filled out by :doc:`API/smacha.templater`
as directed by :doc:`API/smacha.generator` and as determined by
the structure of the :doc:`Scripting/smacha_scripts` parsed by :doc:`API/smacha.parser`.


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

Simply clone into the ``src`` directory of your catkin workspace and run
``catkin_make`` or ``catkin build`` from the root of the workspace. It
may be necessary to update dependencies using ``rosdep`` as follows:

::

   rosdep update
   rosdep install smacha

Usage
=====

Generate
--------

In the simplest case, using default core templates, SMACHA can be
invoked on a ``my_script.yml`` SMACHA script file as follows:

::

   rosrun smacha generate my_script.yml

where the generated code will be output to a file called
``smacha_output.py``

Example usage for the “Nesting State Machines” tutorial:

::

   roscd smacha/test
   rosrun smacha generate smacha_scripts/executive_smach_tutorials/state_machine_nesting2.yml -t smacha_templates/executive_smach_tutorials -o state_machine_nesting2.py -v

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

SMACHA scripts may be directly executed from the command-line
(after being implicitly generated) as follows:

::

   rosrun smacha execute my_script.yml

Example usage for the “Nesting State Machines” tutorial:

::

   roscd smacha/test
   rosrun smacha execute smacha_scripts/executive_smach_tutorials/state_machine_nesting2.yml -t smacha_templates/executive_smach_tutorials -v

Note that the above command requires that ``roscore`` be running.

Again, further arguments and options may be explored by running
``rosrun smacha execute -h`` or ``rosrun smacha execute --help``.

Testing
=======

Tests may be performed by running:

::

   roscd smacha/test

followed by

::

   nosetests executive_smach_tutorials.py

or

::

   python executive_smach_tutorials.py

Passing the ``-h`` argument as follows will bring up a list of some
other options:

::

   python executive_smach_tutorials.py -h

Passing the ``-w`` argument will write the generated output files to
disk, which can subsequently be run as follows, e.g. for the
``state_machine_nesting2.py`` example:

::

   rosrun smacha state_machine_nesting2.yml.py

To run the original file script, use the following command:

::

   rosrun smacha state_machine_nesting2.py

**********
SMACHA API
**********

Modules
=======

.. toctree::
   :hidden:

   The Parser Module<API/smacha.parser>
   The Templater Module<API/smacha.templater>
   The Generator Module<API/smacha.generator>
   The Utilties Module<API/smacha.util>
   The Exceptions Module<API/smacha.exceptions>

These are the main smacha package submodules:

- :doc:`API/smacha.parser`: provides the :class:`smacha.parser.Parser` class for parsing :doc:`Scripting/smacha_scripts`.
- :doc:`API/smacha.templater`: provides the :class:`smacha.templater.Templater` class as well as other functions for processing :doc:`Templating/smacha_templates`.
- :doc:`API/smacha.generator`: provides the :class:`smacha.generator.Generator` class for generating executable Python SMACH code.
- :doc:`API/smacha.util`: provides various utilities.
- :doc:`API/smacha.exceptions`: provides exception definitions.

Templates
=========

.. toctree::
   :hidden:

These are the core smacha package templates:

Base Templates
--------------

.. toctree::
   :hidden:

   The Base Template<API/Templates/Base.tpl.py>

- :doc:`API/Templates/Base.tpl.py`: used for specifying the bare bones of a Python SMACH state machine script.

Container Templates
-------------------

.. toctree::
   :hidden:

   The StateMachine Template<API/Templates/StateMachine.tpl.py>
   The Concurrence Template<API/Templates/Concurrence.tpl.py>

- :doc:`API/Templates/StateMachine.tpl.py`: used for inserting a `StateMachine container <http://wiki.ros.org/smach/Tutorials/StateMachine%20container>`__.
- :doc:`API/Templates/Concurrence.tpl.py`: used for inserting a `Concurrence container <http://wiki.ros.org/smach/Tutorials/Concurrence%20container>`__.


State Templates
---------------

.. toctree::
   :hidden:

   The CallbacksState Template<API/Templates/CallbacksState.tpl.py>
   The PrintUserdataState Template<API/Templates/PrintUserdataState.tpl.py>
   The RandomOutcomeState Template<API/Templates/RandomOutcomeState.tpl.py>
   The ServiceState Template<API/Templates/ServiceState.tpl.py>
   The SimpleActionState Template<API/Templates/SimpleActionState.tpl.py>
   The SleepState Template<API/Templates/SleepState.tpl.py>
   The TF2ListenerState Template<API/Templates/TF2ListenerState.tpl.py>

- :doc:`API/Templates/CallbacksState.tpl.py`: used for creating lambda function callbacks.
- :doc:`API/Templates/PrintUserdataState.tpl.py`: used for printing userdata entries to standard output.
- :doc:`API/Templates/RandomOutcomeState.tpl.py`: used for selecting a random outcome from a specified list of outcomes.
- :doc:`API/Templates/ServiceState.tpl.py`: used for inserting a `ServiceState <http://wiki.ros.org/smach/Tutorials/ServiceState>`__.
- :doc:`API/Templates/SimpleActionState.tpl.py`: used for inserting a `SimpleActionState <http://wiki.ros.org/smach/Tutorials/SimpleActionState>`__.
- :doc:`API/Templates/TF2ListenerState.tpl.py`: used for reading TF2 transforms.

Other Templates
---------------

.. toctree::
   :hidden:

   The State Template<API/Templates/State.tpl.py>
   The Utilities Template<API/Templates/Utils.tpl.py>
   The TF2ListenerSingleton Template<API/Templates/TF2ListenerSingleton.tpl.py>

- :doc:`API/Templates/State.tpl.py`: contains code common to all state templates.
- :doc:`API/Templates/Utils.tpl.py`: contains template macros and other utilities.
- :doc:`API/Templates/TF2ListenerSingleton.tpl.py`: provides a helper class for :doc:`API/Templates/TF2ListenerState.tpl.py`.

******************
Indices and tables
******************

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
