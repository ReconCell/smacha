*****************************
Sub-Scripts and Super-Scripts
*****************************

.. toctree::

**Sub-scripts** may be included from **super-scripts** in order to aid script
reuse and decrease repetition. This functions on the basis of converting
container states into separate sub-scripts that are included in their parent
state machines defined in super-scripts.

Super-Script Example
====================

Looking again at the ``seq_nesting_1.yml`` file from the
``test/smacha_scripts/smacha_test_examples``
folder in the ``smacha`` package, we have:


.. code-block:: yaml

  # SMACHA state sequence example
  name: sm_top
  template: Base
  manifest: smacha
  node_name: smacha_params_test
  outcomes: [final_outcome_a, final_outcome_b, final_outcome_c]
  states:
  - SUB:
      template: StateMachine
      params: {name: FOO_0, outcome: outcome_a, name_1: FOO_1}
      outcomes: [sub_outcome_1, sub_outcome_2]
      transitions: {sub_outcome_1: final_outcome_b, sub_outcome_2: FOO_2}
      states:
      - FOO_0:
          template: ParamFoo
          name_param: [params, name]
          outcome_param: [params, outcome]
          transitions: {outcome_a: FOO_1, outcome_b: sub_outcome_1}
      - FOO_1:
          template: ParamFoo
          name_param: [params, name_1]
          outcome_param: [params, outcome]
          transitions: {outcome_a: sub_outcome_2, outcome_b: sub_outcome_1}
  - FOO_2:
      template: ParamFoo
      params: {name: FOO_2, outcome: outcome_a}
      name_param: [params, name]
      outcome_param: [params, outcome]
      transitions: {outcome_a: final_outcome_a, outcome_b: final_outcome_c}

The ``SUB`` container state may be converted into a sub-script and the
overall script may be converted into a super-script that includes that
sub-script as follows:

.. code-block:: yaml

  # SMACHA state sequence example
  name: sm_top
  template: Base
  manifest: smacha
  node_name: smacha_params_test
  outcomes: [final_outcome_a, final_outcome_b, final_outcome_c]
  states:
  - SUB:
      script: seq_nesting_1_sub_script
      params: {name: FOO_0, outcome: outcome_a, name_1: FOO_1}
      transitions: {sub_outcome_1: final_outcome_b, sub_outcome_2: FOO_2}
  - FOO_2:
      template: ParamFoo
      params: {name: FOO_2, outcome: outcome_a}
      name_param: [params, name]
      outcome_param: [params, outcome]
      transitions: {outcome_a: final_outcome_a, outcome_b: final_outcome_c}

Here, the `script parameters <smacha_scripts.html#script-parameters>`__
``name``, ``outcome`` and ``name_1`` are passed
by the ``SUB`` state to the included ``seq_nesting_1_sub_script`` that defines
it.  This is where the utility of the script parameters becomes clear.

Sub-Script Example
==================

The ``seq_nesting_1_sub_script`` included as the sub-script by the ``SUB``
state in the above super-script is then simply defined in the corresponding
file as the original container state definition:

.. code-block:: yaml

  - SUB:
      template: StateMachine
      outcomes: [sub_outcome_1, sub_outcome_2]
      states:
      - FOO_0:
          template: ParamFoo
          name_param: [params, name]
          outcome_param: [params, outcome]
          transitions: {outcome_a: FOO_1, outcome_b: sub_outcome_1}
      - FOO_1:
          template: ParamFoo
          name_param: [params, name_1]
          outcome_param: [params, outcome]
          transitions: {outcome_a: sub_outcome_2, outcome_b: sub_outcome_1}

SMACHA fills the parameters out with their values as defined in the super-script
when parsing the scripts.

The Extract Tool
================

SMACHA also provides a convenience tool for extracting container states from
a script and coverting them into sub-scripts and super-scripts respectively.

The sub-script and super-script from the above example can be produced by running
the following command on the original ``seq_nesting_1.yml`` script:


::

   rosrun smacha extract `rospack find smacha`/test/smacha_scripts/smacha_test_examples/seq_nesting_1.yml SUB -sub seq_nesting_sub_script.yml -sup seq_nesting_super_script.yml
