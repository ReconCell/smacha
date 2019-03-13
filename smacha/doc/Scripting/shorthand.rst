***********************
Shorthand Script Syntax
***********************

.. toctree::

It is possible to write SMACHA scripts in `shorthand
<https://en.wikipedia.org/wiki/Shorthand>`_ syntax in order to increase script
brevity. The :class:`Parser module <smacha.parser.Parser>` provides two methods,
:func:`smacha.parser.Parser.sub_longhand` and :func:`smacha.parser.Parser.sub_shorthand`
for converting scripts back and forth between the longhand and shorthand formats.

Shorthand Variable Names
========================

Core variable names may be abbreviated in shorthand syntax as follows:

* ``name``: ``n``,
* ``manifest``: ``m``,
* ``node_name``: ``nn``,
* ``template``: ``T``,
* ``script``: ``S``,
* ``outcomes``: ``o``,
* ``states``: ``s``,
* ``params``: ``p``,
* ``userdata``: ``ud``,
* ``input_keys``: ``ik``,
* ``output_keys``: ``ok``,
* ``remapping``: ``r``,
* ``transitions``: ``t``,
* ``default_outcome``: ``do``,
* ``outcome_map``: ``om``,
* ``callbacks``: ``cb``.

Example usage of these abbreviations is provided in the
:ref:`SMACHA documentation introduction <shorthand-example>`, as well
as in numerous test scripts provided in the
`test/smacha_scripts/smacha_test_examples <https://gitlab.com/reconcell/smacha/tree/master/smacha/test/smacha_scripts/smacha_test_examples>`_
folder of the `smacha <https://gitlab.com/reconcell/smacha/tree/master/smacha>`_ package

Shorthand Parameter Insertion
=============================

Parameter insertions can also be abbreviated using shorthand syntax.
For example, if the script parameters ``subject``, ``verb``
and ``object`` contained the values ``I``, ``writing`` and ``documentation``
respectively, then we can form the construction

.. code-block:: yaml

  [[[subject]], 'love', [[verb]], [[object]]]

using shorthand syntax, instead of

.. code-block:: yaml

  [[params, subject], 'love', [params, verb], [params, object]]

in longhand syntax, both of which would be parsed as ``I love writing documentation``.
