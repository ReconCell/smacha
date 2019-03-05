********************
Template Inheritance
********************

.. toctree::

.. note:: Before reading the following documentation, it is highly recommended
          that you consult the `"Jinja2 template inheritance documentation
          <http://jinja.pocoo.org/docs/2.10/templates/#template-inheritance>`__
          first!

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
