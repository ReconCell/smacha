****************
SMACHA Templates
****************

`Jinja2 <http://jinja.pocoo.org/docs/2.9/>`__ is a powerful template
engine for Python. Jinja2 templates are used to specify how code should
be generated from SMACHA scripts. The ``Base`` template from the above
example is specified in a ``Base.tpl`` file and looks like this:

.. code:: python

   {% from "Utils.tpl" import render_userdata %}
   {% set defined_headers = [] %}
   {% block base_header %}
   #!/usr/bin/env python
   {{ base_header }}
   {% endblock base_header %}

   {% block imports %}
   import roslib; {% if manifest is defined %}roslib.load_manifest('{{ manifest }}'){% endif %}
   import rospy
   import smach
   import smach_ros
   {{ imports }}
   {% endblock imports %}

   {% block defs %}
   {{ defs }}
   {% endblock defs %}

   {% block class_defs %}
   {{ class_defs }}
   {% endblock class_defs %}

   {% if name is defined %}{% set sm_name = name | lower() %}{% else %}{% set sm_name = 'sm' %}{% endif %}

   {% block main_def %}
   def {% if function_name is defined %}{{ function_name | lower() }}{% else %}main{% endif %}():
       rospy.init_node('{% if node_name is defined %}{{ node_name }}{% else %}{{ name }}{% endif %}')

       {{ main_def | indent(4) }}
   {% endblock main_def %}
      
   {% block body %}
       {{ sm_name }} = smach.StateMachine(outcomes=[{% for outcome in outcomes %}'{{ outcome }}'{% if not loop.last %}, {% endif %}{% endfor %}])

       {% if userdata is defined %}{{ render_userdata(name | lower(), userdata) | indent(4) }}{% endif %}
       {% if name in header %}{{ header[name] | indent(4, true) }}{% endif %}

       with {{ sm_name }}:

           {{ body | indent(8) }}
   {% endblock body %}

   {% block footer %}
           {{ footer | indent(8) }}
   {% endblock footer %}

   {% block introspection_server %}
       sis = smach_ros.IntrospectionServer('{% if node_name is defined %}{{ node_name }}{% else %}{{ name }}{% endif %}', {{ name | lower() }}, '/{{ name }}')
       sis.start()
   {% endblock introspection_server %}

   {% block execute %}
       {{ execute | indent(4) }}

       outcome = {{ sm_name }}.execute()
   {% endblock execute %}    

   {% block spin %}   
       rospy.spin()
   {% endblock spin %}

   {% block base_footer %}
       {{ base_footer | indent(4) }}
   {% endblock base_footer %}

   {% block main %}
   if __name__ == '__main__':
   {{ '' | indent(4, true) }}{% if function_name is defined %}{{ function_name | lower() }}{% else %}main{% endif %}()
   {% endblock main %}

Core Templates
==============

SMACHA provides default core templates for many of the SMACH states and
containers, as well as for other useful constructs.

So far, the following core templates are present and functional:

-  ``Base.tpl.py``: the core base template used for specifying the bare
   (bones) of a a Python SMACH state machine script.

-  ``State.tpl``: contains functionality common to all states,
   e.g. userdata specification.

-  ``StateMachine.tpl``: the core template used for inserting a
   `StateMachine
   container <http://wiki.ros.org/smach/Tutorials/StateMachine%20container>`__.

-  ``Concurrence.tpl``: the core template used for inserting a
   `Concurrence
   container <http://wiki.ros.org/smach/Tutorials/Concurrence%20container>`__.

-  ``ServiceState.tpl``: the core template used for inserting a
   `ServiceState <http://wiki.ros.org/smach/Tutorials/ServiceState>`__.

-  ``SimpleActionState.tpl``: the core template used for inserting a
   `SimpleActionState <http://wiki.ros.org/smach/Tutorials/SimpleActionState>`__.

-  ``TF2ListenerState.tpl``: used for reading TF2 transforms.

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
