***************
Template Macros
***************

The :doc:`Utilities template <../API/Templates/Utils.tpl.py>` contains many
`template macros <http://jinja.pocoo.org/docs/2.10/templates/#macros>`_
designed to make repetitive template tasks easier:

* ``render_userdata(namespace, userdata)``: a macro for rendering ``userdata`` state machine headers,

* ``render_outcomes(outcomes, indent=0)``: a macro for rendering ``outcomes`` in state instantiations,

* ``render_input_keys(input_keys, indent=37)``: a macro for rendering ``input_keys`` in state instantiations,

* ``render_output_keys(output_keys, indent=37)``: a macro for rendering ``output_keys`` in state instantiations,

* ``render_goal_slots(goal_slots, indent=51)``: a macro for rendering ``goal_slots`` in state instantiations,

* ``render_result_slots(result_slots, indent=51)``: a macro for rendering ``result_slots`` in state instantiations,

* ``render_request_slots(request_slots, indent=51)``: a macro for rendering ``request_slots`` in state instantiations,

* ``render_response_slots(response_slots, indent=51)``: a macro for rendering ``response_slots`` in state instantiations,

* ``render_def_lambda_callbacks(defined_headers, name, input_keys, callbacks)``: a macro for rendering ``callbacks`` lambda callback definitions,

* ``render_init_callbacks()``: a macro for rendering ``callbacks`` initialization in state class ``__init__()`` methods,

* ``render_execute_callbacks()``: a macro for rendering ``callbacks`` execution in state class ``execute()`` methods,

* ``render_callbacks(name, callbacks, indent=51)``: a macro for rendering ``callbacks`` dicts in state instantiations,

* ``render_outcome_map(outcome_map, indent=35)``: a macro for rendering ``outcome_map`` in state instantiations,

* ``render_transitions(transitions, indent=23)``: a macro for rendering ``transitions`` in state machine ``add()`` methods,

* ``render_remapping(remapping, indent=23)``: a macro for rendering ``remapping`` in state machine ``add()`` methods,

* ``import_module(defined_headers, module)``: a macro for importing a module,

* ``import_module_as(defined_headers, module, new_name)``: a macro for importing a module and renaming it as something else,

* ``from_import(defined_headers, module, name)``: a macro for importing a name from a module,

* ``from_import_as(defined_headers, module, name, new_name)``: a macro for importing a name from a module and renaming it as something else.
