********
Userdata
********

.. toctree::

.. note:: Before reading the following documentation, it is highly recommended
          that you consult the `"Passing User Data between States"
          <https://wiki.ros.org/smach/Tutorials/User%20Data>`__ SMACH tutorial
          first!

**Userdata** is a special type of data structure that is inherited from SMACH
and is used to pass data between states as either **input_keys** or **output_keys**.

Certain functionalities have been developed for SMACHA templates to make it easy to
manipulate userdata within scripts.

Userdata Hello World Example
============================

Here is the :download:`print_userdata.yml </../test/smacha_scripts/smacha_test_examples/print_userdata.yml>` script from the
`test/smacha_scripts/smacha_test_examples <https://gitlab.com/reconcell/smacha/tree/master/smacha/test/smacha_scripts/smacha_test_examples>`__
folder in the `smacha <https://gitlab.com/reconcell/smacha/tree/master/smacha>`__ package
that develops a simple "Hello World" example for userdata manipulation:

.. literalinclude:: /../test/smacha_scripts/smacha_test_examples/print_userdata.yml
   :language: yaml

In this example a variable ``foo`` with the value 'Hello World!' is added to the
userdata at the beginning of the script and is passed as an `input_key` to the
``FOO_0`` state which uses a :doc:`PrintUserdataState template
<../API/Templates/PrintUserdataState.tpl.py>` which looks like this:

.. literalinclude:: /../src/smacha/templates/PrintUserdataState.tpl.py
   :language: python

This template defines a small class that simply prints out any `input_keys` that it is
passed to standard output.

Userdata Definitions
====================

In the subsequent ``FOO_1`` state, which uses a :doc:`CallbacksState template
<../API/Templates/CallbacksState.tpl.py>`, another variable called ``bar`` is
entered into the userdata with the value `Goodbye World!'. The
:doc:`CallbacksState template <../API/Templates/CallbacksState.tpl.py>` is
interesting in its own right and can be quite powerful in other circumstances,
but it is only used in this example as an intermediary state for illustrative
purposes.

.. note:: All userdata that is defined within a script is entered into the
          userdata structure **at the beginning of run-time** when the state
          machine is first run. This is a carryover from the way in which SMACH
          state machines are designed. However, it can still be useful for a
          script designer to be able to define userdata at arbitrary points
          within a script, e.g. within a the state that is going to make use of
          it, as in the above example for the ``FOO_1`` state:

.. code-block:: yaml

    - FOO_1:
        template: CallbacksState
        userdata: {bar: 'Goodbye World!'}
        transitions: {succeeded: FOO_2}

Wherever the userdata definitions are located within the script, be that within
the main script, or in a container state or sub-script, SMACHA will try to place
them at the appropriate locations in the rendered SMACH Python code.
This "smart" rendering of userdata will be more apparent in later examples.

Userdata Remappings
===================

After ``FOO_1`` in the above example, the state machine transitions to ``FOO_2``,
but according to its definition within the script (admittedly contrived for illustrative purposes!),
``FOO_2`` expects an `input_key` named ``foobar``, yet the userdata only contains
the ``foo`` and ``bar`` variables.

This is easily solved with a **remapping** (another carryover from SMACH),
which allows for the ``bar`` variable to be mapped onto the ``foobar``
`input_key`:

.. code-block:: yaml

  - FOO_2:
      template: PrintUserdataState
      input_keys: [foobar]
      remapping: {foobar: bar}
