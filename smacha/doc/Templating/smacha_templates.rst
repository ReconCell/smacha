****************
SMACHA Templates
****************

.. toctree::

.. note:: Before reading the following documentation, particularly if you are
          unfamiliar with templating, it is highly recommended that you consult
          the `"Jinja2 documentation <http://jinja.pocoo.org/>`__ first!

**SMACHA templates** are used to specify how SMACH Python code should be
generated from SMACHA scripts. They are effectively code skeletons with
placeholder variables that are fleshed out with the contents of the scripts via
the recursive code generation process. The SMACHA API renders these templates
using the `Jinja2 <http://jinja.pocoo.org/docs/2.9/>`__ library, a powerful
template engine for Python, coupled with some custom modifications to its usual
rendering behaviour in order to suit this particular use case.

Base Templates
==============

All SMACHA scripts start by specifying a ``base template``,
which is like the spine of the code skeleton that serves as a
starting point from which the code generation process can start
to fill in the meat of the code.

Looking again at :download:`seq.yml
</../test/smacha_scripts/smacha_test_examples/seq.yml>` script from the
:ref:`Linear State Sequence Example <linear-state-seq-example>` in the
:doc:`Scripting Tutorial <../Scripting/smacha_scripts>`,
we see that the base template ``Base`` is specified at the top of the script:

.. literalinclude:: /../test/smacha_scripts/smacha_test_examples/seq.yml
   :language: yaml
   :lines: 1-3

This ``Base`` template is the :doc:`core SMACHA Base template <../API/Templates/Base.tpl.py>`
defined in the :download:`Base.tpl.py </../src/smacha/templates/Base.tpl.py>` file.

.. note:: The `.tpl.py` extension indicates that it is a template, while still
          allowing text editors to invoke their Python syntax highlighters.

The :download:`Base.tpl.py </../src/smacha/templates/Base.tpl.py>` file looks like this:

.. literalinclude:: /../src/smacha/templates/Base.tpl.py
   :language: python

Core Code Generation Variables and Code Blocks
==============================================

There are a number of core code generation variables and code blocks
present in the core templates that enable the code generation engine to
produce code in the appropriate places.

-  ``base_header`` block: used to specify any code that must appear near
   the top of the program script.

-  ``defs`` block: used to position function definitions.

-  ``class_defs`` block: used to position class definitions.

-  ``main_def`` block: used to position the main function definition.

-  ``header`` block: the ``header`` block in a state template is
   rendered into the ``header`` variable of either its parent template
   or the base template depending on its nesting depth.

-  ``body`` block: The ``body`` block in a state template is rendered
   into the ``body`` variable of either its parent state or the base
   template depending on its nesting depth.

-  ``footer`` block: The ``footer`` block in a state template is
   rendered into the ``footer`` variable of either its parent template
   or the base template depending on its nesting depth.

-  ``execute`` block: used to position the code necessary for executing
   the state machine.

-  ``base_footer`` block: used to specify any code that must appear near
   the bottom of the program script.

-  ``main`` block: used to specify the code necessary to execute the
   main function.

Some additional blocks may be optionally included, such as the
*introspection_server* and ROS *spin* blocks, if an introspection server
is required for use with the SMACH viewer, or *comment* blocks, used to
decorate the generated source code.

Note that all of the above code generation variables and code blocks may
be either removed, modified or arbitrarily customized within the API for
particular use-cases. The code insertion order may also be specified
within the API, i.e. code may be either prepended or appended to a
variable.

Overriding Core Templates, Variables and Blocks via Template Inheritance
========================================================================

Jinja2 provides powerful template functionality, including the ability
to extend templates via `template
inheritance <http://wiki.ros.org/smach/Tutorials/SimpleActionState>`__,
such that their constituent code blocks may be overridden or extended as
required. SMACHA aims to incorporate as much of this functionality as
possible, thus the core templates may be overridden or augmented by
custom user-specified templates via the usual Jinja2 template
inheritance mechanisms, with some caveats.

This works in the usual way using the following Jinja2 variables and
expressions:

-  ``{% extends "<template_name>" %}``: When this expression appears at
   the top of a template, the template will inherit code blocks from the
   parent template specified by ``<template_name>``.

-  ``{{ super() }}``: When this expression appears inside a block, the
   code from the same block in the parent template as specified by
   ``{% extends %}`` will be rendered at its position.

-  ``{% include "<template_name>" %}``: When this expression appears at
   the top of a template, the template will include all code from the
   template specified by ``<template_name>``.

Caveats: if a state template contains blocks, but does not contain an
``{{ extends }}`` expression at the top of a template, it is implied
that the code for the blocks will be rendered into variables and blocks
with the same names as the blocks in the state template as dictated by
the SMACHA script and as defined usually either by the base template or
container templates. This behaviour is specific to SMACHA and is not
present in Jinja2. In the current implementation, only base templates
use the ``{% extends %}`` inheritance mechanism, whereas state and
container templates use the ``{% include %}`` mechanism to inherit code
from other templates. See the `Core Code Generation Variables and Code
Blocks Section <#core-code-generation-variables-and-code-blocks>`__ for
examples of how this behaviour works with core code generation variables
and blocks.

See the `Usage Section <#Usage>`__ below for an example of how such
custom templates may be included when generating code via the
command-line in practice.
