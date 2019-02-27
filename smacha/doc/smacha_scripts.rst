**************
SMACHA Scripts
**************

SMACHA scripts are YAML files that describe how SMACHA should generate
SMACH code. Here is the `“Nesting State Machines”
example <http://wiki.ros.org/smach/Tutorials/Nesting%20State%20Machines>`__
from the `SMACH Tutorials <http://wiki.ros.org/smach/Tutorials>`__
described in a SMACHA script:

.. code:: yaml

   --- # Nesting State Machines Tutorial SMACHA script.
   name: sm_top
   template: Base
   manifest: smach_tutorials
   node_name: smach_example_state_machine
   outcomes: [outcome5]
   states:
     - BAS:
         template: Bas
         transitions: {outcome3: SUB}
     - SUB:
         template: StateMachine
         outcomes: [outcome4]
         transitions: {outcome4: outcome5}
         states:
           - FOO:
               template: Foo
               transitions: {outcome1: BAR, outcome2: outcome4}
           - BAR:
               template: Bar
               transitions: {outcome1: FOO}

This demonstrates a reduction from 80 lines of raw SMACH code to 20
lines of SMACHA script. Nice.

==============
Base Variables
==============

The base of the script specifies the following variables:

-  ``name``: a name for the overall state machine,
-  ``template``: the name of its base template,
-  ``manifest`` (optional): a name for an optional ROS manifest,
-  ``node_name``: a name for its associated ROS node,
-  ``outcomes``: a list of its possible outcomes,
-  ``states``: list of its constituent states.

Each of the states in the base script may, in turn, specify similar
variables of their own, as discussed in the following sub-sections.

======
States
======

Each state, including the base, must specify a template from which its
respective code should be generated. States may be specified as lists
specifying their transition order and may also be
`nested <http://wiki.ros.org/smach/Tutorials/Nesting%20State%20Machines>`__
as described in the SMACH tutorials using appropriate combinations of
template and state specifications as seen in the example above.

Outcomes
^^^^^^^^

Possible state outcomes may be specified as a list in the base state
machine and in each container state.

Transitions
^^^^^^^^^^^

Possible state transitions may be specified as an associative array
(hash/dictionary) in each state.

Remappings
^^^^^^^^^^

Input and output remappings of user data (not shown in the above
example; see the `SMACH User Data
Tutorial <http://wiki.ros.org/smach/Tutorials/User%20Data>`__ for more
details) may be specified as an associative array in each state.
