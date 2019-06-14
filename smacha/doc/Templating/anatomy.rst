****************
Template Anatomy
****************

.. toctree::

The :download:`Base.tpl.py </../src/smacha/templates/Base.tpl.py>` file looks like this:

.. literalinclude:: /../src/smacha/templates/Base.tpl.py
   :language: python

The Meta Block
==============

Core Variables
==============

Some variables are quite important for core template functioning:

- the ``name`` variable: this is always filled with the state name from the script,

- the ``defined_headers`` variable: this is a list that is defined at the top of the ``Base`` template
  and is used to keep a record of the names of items that have been defined throughout the code
  generation process such that they are not re-defined, e.g. module imports,

- the ``local_vars`` variable: this is a list that is defined at the top of
  various templates that is used to contain variable names in templates that
  should be non-persistent between states (i.e. local). Variables that are set
  in templates that are not contained in this list are assumed to be persistent
  between states (i.e. global) by default.

Core Blocks and Code Insertion Variables
========================================

There are a number of core blocks and code insertion variables present in the
``Base`` template above, as well as in many of the other :doc:`core API state
templates <core_api_state_templates>`, that enable the code generation engine to
produce code in the appropriate places:

-  the ``base_header`` block: used to specify any code that must appear near
   the top of the program script,

-  the ``imports`` block: used to position module imports,

-  the ``defs`` block: used to position function definitions,

-  the ``class_defs`` block: used to position class definitions,

-  the ``main_def`` block: used to position the main function definition,

-  the ``header`` block: the ``header`` block in a state template is
   rendered into the ``header`` variable of either its parent template
   or the base template depending on its nesting depth,

-  the ``body`` block: the ``body`` block in a state template is rendered
   into the ``body`` variable of either its parent state or the base
   template depending on its nesting depth,

-  the ``footer`` block: the ``footer`` block in a state template is
   rendered into the ``footer`` variable of either its parent template
   or the base template depending on its nesting depth,

-  the ``execute`` block: used to position the code necessary for executing
   the state machine,

-  the ``base_footer`` block: used to specify any code that must appear near
   the bottom of the program script,

-  the ``main`` block: used to specify the code necessary to execute the
   main function.

Some additional blocks may be optionally included, such as the
*introspection_server* and ROS *spin* blocks, if an introspection server
is required for use with the SMACH viewer, or *comment* blocks, used to
decorate the generated source code.

.. note:: All of the above code insertion variables and code blocks may be
          either removed, modified or arbitrarily customized within the API for
          particular use cases. The code insertion order may also be specified
          within the API, i.e. code may be either prepended or appended to a
          variable.
