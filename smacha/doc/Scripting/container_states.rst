****************
Container States
****************

.. toctree::

.. note:: Before reading the following documentation, it is highly recommended
          that you consult the `"Create a Hierarchical State Machine"
          <https://wiki.ros.org/smach/Tutorials/Create%20a%20hierarchical%20state%20machine>`__
          and `"Concurrent State Machine"
          <https://wiki.ros.org/smach/Tutorials/Concurrent%20States>`__ SMACH
          tutorials first!

**Container states** are special types of states that
allow for state constructions to extend beyond linear state sequences.
SMACHA uses two types of container states: **StateMachine**
containers and **Concurrence** containers, for producing
nested state machine hierarchies and parallel state machines respectively.

.. _nested-state-machine-example:

Nested State Machine Example
============================

Here is a simple SMACHA script :download:`seq_nesting_1.yml
</../test/smacha_scripts/smacha_test_examples/seq_nesting_1.yml>` from the
`test/smacha_scripts/smacha_test_examples <https://gitlab.com/reconcell/smacha/tree/master/smacha/test/smacha_scripts/smacha_test_examples>`__
folder in the `smacha <https://gitlab.com/reconcell/smacha/tree/master/smacha>`__ package
that uses a `StateMachine` container to define a nested state hierarchy:

.. literalinclude:: /../test/smacha_scripts/smacha_test_examples/seq_nesting_1.yml
   :language: yaml

If we execute this script by running the following command in one terminal:

::

   rosrun smacha execute -t `rospack find smacha`/test/smacha_templates/smacha_test_example -i -v `rospack find smacha`/test/smacha_scripts/smacha_test_examples/seq_nesting_1.yml

and run `smach_viewer` from the `executive_smach_visualization
<http://wiki.ros.org/executive_smach_visualization>`__ package in another
terminal:

::

   rosrun smach_viewer smach_viewer.py

we get the following visualization of the resulting state machine:

.. |pic1| image:: ../_static/seq_nesting_1_depth_0.png
   :width: 45%

.. |pic2| image:: ../_static/seq_nesting_1_depth_1.png
   :width: 45%

.. table:: Nested state machine example visualized at two different depth levels.

  +---------+---------+
  | |pic1|  | |pic2|  |
  +---------+---------+


Concurrent State Machines
=========================

Here is a simple SMACHA script :download:`seq_concurrence_1.yml
</../test/smacha_scripts/smacha_test_examples/seq_concurrence_1.yml>` from the
`test/smacha_scripts/smacha_test_examples <https://gitlab.com/reconcell/smacha/tree/master/smacha/test/smacha_scripts/smacha_test_examples>`__
folder in the `smacha <https://gitlab.com/reconcell/smacha/tree/master/smacha>`__ package
that uses a `Concurrence` container to define state machines that run in parallel:

.. literalinclude:: /../test/smacha_scripts/smacha_test_examples/seq_concurrence_1.yml
   :language: yaml

If we execute this script by running the following command in one terminal:

::

   rosrun smacha execute -t `rospack find smacha`/test/smacha_templates/smacha_test_example -i -v `rospack find smacha`/test/smacha_scripts/smacha_test_examples/seq_concurrence_1.yml

and run `smach_viewer` from the `executive_smach_visualization
<http://wiki.ros.org/executive_smach_visualization>`__ package in another
terminal:

::

   rosrun smach_viewer smach_viewer.py

we get the following visualization of the resulting state machine:

.. |pic3| image:: ../_static/seq_concurrence_1_depth_0.png
   :width: 45%

.. |pic4| image:: ../_static/seq_concurrence_1_depth_1.png
   :width: 45%

.. table:: Concurrent state machines example visualized at two different depth levels.

  +---------+---------+
  | |pic3|  | |pic4|  |
  +---------+---------+

.. _contain-tool:

The Contain Tool
================

Since it can be cumbersome to convert linear state sequences to nested `StateMachine`
container states or parallel `Concurrence` container states, either in SMACHA YAML
script or, indeed, in raw SMACH Python code, SMACHA provides a utility
called the **contain tool** in order to automate this process.

In order to illustrate how it works, in the following, we will use the
:download:`seq.yml </../test/smacha_scripts/smacha_test_examples/seq.yml>` script from the
`test/smacha_scripts/smacha_test_examples <https://gitlab.com/reconcell/smacha/tree/master/smacha/test/smacha_scripts/smacha_test_examples>`__
folder in the `smacha <https://gitlab.com/reconcell/smacha/tree/master/smacha>`__ package
familiar from the previous sections that defines a linear sequence of states:

.. literalinclude:: /../test/smacha_scripts/smacha_test_examples/seq.yml
   :language: yaml

StateMachine Containerization Example
-------------------------------------

In order to convert the ``FOO_0, FOO_1, FOO_2`` sequence in the above script
into a script with a `StateMachine` state named ``SUB`` containing the sequence
``FOO_0, FOO_1`` followed by ``FOO_2``, we run the following command:

::

   rosrun smacha contain `rospack find smacha`/test/smacha_scripts/smacha_test_examples/seq.yml SUB StateMachine FOO_0 FOO_1 -o seq_nesting_1_contain_output.yml

that produces the following result in the
:download:`seq_nesting_1_contain_output.yml
</../test/smacha_generated_scripts/smacha_test_examples/seq_nesting_1_contain_output.yml>`
generated output file:

.. literalinclude:: /../test/smacha_generated_scripts/smacha_test_examples/seq_nesting_1_contain_output.yml
   :language: yaml

which should match the nested state machine example above.

Concurrence Containerization Example
------------------------------------

In order to convert the ``FOO_0, FOO_1, FOO_2`` sequence in the ``seq.yml``
script into a script with a `Concurrence` state named ``CON`` containing the
parallel states ``FOO_0`` and ``FOO_1`` followed by ``FOO_2``, we run the
following command:

::

   rosrun smacha contain `rospack find smacha`/test/smacha_scripts/smacha_test_examples/seq.yml CON Concurrence FOO_0 FOO_1 -o seq_concurrence_1_contain_output.yml

that produces the following result in the
:download:`seq_concurrence_1_contain_output.yml
</../test/smacha_generated_scripts/smacha_test_examples/seq_concurrence_1_contain_output.yml>`
generated output file:

.. literalinclude:: /../test/smacha_generated_scripts/smacha_test_examples/seq_concurrence_1_contain_output.yml
   :language: yaml

which should match the concurrent state machines example above.
