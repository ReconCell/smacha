SMACHA Documentation
====================

SMACHA is a `YAML <http://yaml.org/>`__ and
`Jinja2 <http://jinja.pocoo.org/docs/2.9/>`__-based meta-scripting,

templating, and code generation engine for
`SMACH <http://wiki.ros.org/smach>`__.

`SMACH <http://wiki.ros.org/smach>`__ is an exceptionally useful and
comprehensive task-level architecture for state machine construction in
`ROS <http://wiki.ros.org/>`__-based robot control systems. However,
while it provides much in terms of power and flexibility, its overall
task-level simplicity can often be obfuscated at the script-level by
boilerplate code, intricate structure and lack of code reuse between
state machine prototypes.

SMACHA (short for “State Machine Assembler”, pronounced “smasha”) aims
at distilling the task-level simplicity of SMACH into compact YAML-based
scripts in the foreground, while retaining all of its power and
flexibility in Jinja2-based templates and a custom code generation
eng

Overview
--------

.. figure:: _static/smacha_overview.png
   :alt: SMACHA API Overview

   SMACHA API Overview

Scripting
^^^^^^^^^

.. toctree::
   :hidden:

   smacha_scripts

:doc:`smacha_scripts` are YAML files that describe how SMACHA should generate
SMACH code. They are parsed by :doc:`smacha.parser` and direct
:doc:`smacha.generator` on how to combine :doc:`smacha_templates` using the
:doc:`smacha.templater`.

Templating
^^^^^^^^^^

.. toctree::
   :hidden:

   smacha_templates

:doc:`smacha_templates` are Jinja2-based Python SMACH code templates
containing variables that are filled out by :doc:`smacha.templater`
as directed by :doc:`smacha.generator` and as determined by
the structure of the :doc:`smacha_scripts` parsed by :doc:`smacha.parser`.


Code Generation
^^^^^^^^^^^^^^^

.. toctree::
   :hidden:

   smacha_code_generator

The :doc:`smacha_code_generator` recursively processes :doc:`smacha_scripts`
parsed by :doc:`smacha.parser`, manages the use of :doc:`smacha_templates` that
are filled out by :doc:`smacha.templater`, and renders the final result to
executable Python SMACH code.

Installation
------------

Simply clone into the ``src`` directory of your catkin workspace and run
``catkin_make`` or ``catkin build`` from the root of the workspace. It
may be necessary to update dependencies using ``rosdep`` as follows:

::

   rosdep update
   rosdep install smacha

Usage
-----

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

Testing
-------

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

smacha package API
==================

.. toctree::
   :hidden:

   smacha.parser
   smacha.templater
   smacha.generator
   smacha.util
   smacha.exceptions

These are the main smacha package submodules:

- :doc:`smacha.parser`: provides the :class:`smacha.parser.Parser` class for parsing :doc:`smacha_scripts`.
- :doc:`smacha.templater`: provides the :class:`smacha.templater.Templater` class as well as other functions for processing :doc:`smacha_templates`.
- :doc:`smacha.generator`: provides the :class:`smacha.generator.Generator` class for generating executable Python SMACH code.
- :doc:`smacha.util`: provides various utilities.
- :doc:`smacha.exceptions`: provides exception definitions.

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
