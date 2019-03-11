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

Looking again at the 
:download:`seq_nesting_1.yml </../test/smacha_scripts/smacha_test_examples/seq_nesting_1.yml>` script from the
`test/smacha_scripts/smacha_test_examples <https://gitlab.com/reconcell/smacha/tree/master/smacha/test/smacha_scripts/smacha_test_examples>`__
folder in the `smacha <https://gitlab.com/reconcell/smacha/tree/master/smacha>`__ package, we have:

.. literalinclude:: /../test/smacha_scripts/smacha_test_examples/seq_nesting_1.yml
   :language: yaml

The ``SUB`` container state may be converted into a sub-script and the
overall script may be converted into a super-script that includes that
sub-script as follows:

.. literalinclude:: /../test/smacha_scripts/smacha_test_examples/seq_nesting_1_super_script.yml
   :language: yaml

Here, the :ref:`script parameters <script-params>` ``name``, ``outcome`` and
``name_1`` are passed by the ``SUB`` state to the included
``seq_nesting_1_sub_script`` that defines it. This is where the utility of the
script parameters becomes clear.

Sub-Script Example
==================

The ``seq_nesting_1_sub_script`` included as the sub-script by the ``SUB``
state in the above super-script is then simply defined in the corresponding
file as the original container state definition:

.. literalinclude:: /../test/smacha_scripts/smacha_test_examples/seq_nesting_1_sub_script.yml
   :language: yaml

SMACHA fills the parameters out with their values as defined in the super-script
when parsing the scripts.

.. _extract-tool:

The Extract Tool
================

SMACHA also provides a convenience tool for extracting container states from
a script and coverting them into sub-scripts and super-scripts respectively.

The sub-script and super-script from the above example can be produced by running
the following command on the original ``seq_nesting_1.yml`` script:


::

   rosrun smacha extract `rospack find smacha`/test/smacha_scripts/smacha_test_examples/seq_nesting_1.yml SUB -sub seq_nesting_sub_script_extract_output.yml -sup seq_nesting_super_script_extract_output.yml

that produces the following super-script in the
:download:`seq_nesting_1_super_script_extract_output.yml </../test/smacha_generated_scripts/smacha_test_examples/seq_nesting_1_super_script_extract_output.yml>`
generated output file:

.. literalinclude:: /../test/smacha_generated_scripts/smacha_test_examples/seq_nesting_1_super_script_extract_output.yml
   :language: yaml

and the following sub-script in the
:download:`seq_nesting_1_sub_script_extract_output.yml </../test/smacha_generated_scripts/smacha_test_examples/seq_nesting_1_sub_script_extract_output.yml>`
generated output file:

.. literalinclude:: /../test/smacha_generated_scripts/smacha_test_examples/seq_nesting_1_sub_script_extract_output.yml
   :language: yaml

which should match the nested state machine super-script and sub-script examples above.
