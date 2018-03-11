{% block meta %}
name: Utils
description: SMACH template containing macros commonly used by other templates.
language: Python
framework: SMACH
type: None
tags: [core]
includes: []
extends: []
variables: []
input_keys: []
output_keys: []
{% endblock meta %}

#
# Macro for rendering 'userdata' state machine headers.
#
{% macro render_userdata(namespace, userdata) %}
{% for userdatum_key, userdatum_val in userdata.items() | sort %}
{{ namespace | lower() }}.userdata['{{ userdatum_key | lower() }}'] = {% if userdatum_val is not_string %}{{ userdatum_val }}{% else %}'{{ userdatum_val }}'{% endif %}
{% endfor %}
{% endmacro %}

#
# Macro for rendering 'outcomes' in state instantiations.
#
{% macro render_outcomes(outcomes, indent=0) %}{{ 'outcomes=' | indent(indent, true) }}[{% for outcome in outcomes %}'{{ outcome }}'{% if not loop.last %}, {% endif %}{% endfor %}]{% endmacro %}

#
# Macro for rendering 'input_keys' in state instantiations.
#
{% macro render_input_keys(input_keys, indent=37) %}{{ 'input_keys = ' | indent(indent, true) }}[{% for input_key in input_keys %}'{{ input_key }}'{% if not loop.last %}, {% endif %}{% endfor %}]{% endmacro %}

#
# Macro for rendering 'output_keys' in state instantiations.
#
{% macro render_output_keys(output_keys, indent=37) %}{{ 'output_keys = ' | indent(indent, true) }}[{% for input_key in output_keys %}'{{ input_key }}'{% if not loop.last %}, {% endif %}{% endfor %}]{% endmacro %}

#
# Macro for rendering 'goal_slots' in state instantiations.
#
{% macro render_goal_slots(goal_slots, indent=51) %}{{ 'goal_slots = ' | indent(indent, true) }}[{% for goal_slot in goal_slots %}'{{ goal_slot }}'{% if not loop.last %}, {% endif %}{% endfor %}]{% endmacro %}

#
# Macro for rendering 'result_slots' in state instantiations.
#
{% macro render_result_slots(result_slots, indent=51) %}{{ 'result_slots = ' | indent(indent, true) }}[{% for result_slot in result_slots %}'{{ result_slot }}'{% if not loop.last %}, {% endif %}{% endfor %}]{% endmacro %}

#
# Macro for rendering 'request_slots' in state instantiations.
#
{% macro render_request_slots(request_slots, indent=51) %}{{ 'request_slots = ' | indent(indent, true) }}[{% for request_slot in request_slots %}'{{ request_slot }}'{% if not loop.last %}, {% endif %}{% endfor %}]{% endmacro %}

#
# Macro for rendering 'response_slots' in state instantiations.
#
{% macro render_response_slots(response_slots, indent=51) %}{{ 'response_slots = ' | indent(indent, true) }}[{% for response_slot in response_slots %}'{{ response_slot }}'{% if not loop.last %}, {% endif %}{% endfor %}]{% endmacro %}

#
# Macro for rendering 'callbacks' lambda callback definitions.
#
{% macro render_def_lambda_callbacks(defined_headers, name, input_keys, callbacks) %}
{% for cb_output_key, cb in callbacks.iteritems() %}
{% if cb is expression %}
{% set cb_name = cb_output_key|lower + '_' + name|lower + '_lambda_cb' %}
{% if cb_name not in defined_headers %}
@smach.cb_interface(input_keys=[{% for input_key in input_keys %}'{{ input_key }}'{% if not loop.last %}, {% endif %}{% endfor %}], 
                    output_keys=['{{ cb_output_key }}'],
                    outcomes=['succeeded'])
def {{ cb_name }}(userdata):
    lambda_cb = {{ cb | exptostr }}
    userdata['{{ cb_output_key }}'] = lambda_cb(userdata)
    return 'succeeded'
{% do defined_headers.append(cb_name) %}{% endif %}
{% endif %}
{% endfor %}
{% endmacro %}

#
# Macro for rendering 'callbacks' initialization in state class __init__() methods.
#
{% macro render_init_callbacks() %}
        self._cbs = []
        for cb in sorted(callbacks):
            if cb in globals():
                self._cbs.append(globals()[cb])

        self._cb_input_keys = []
        self._cb_output_keys = []
        self._cb_outcomes = []

        for cb in self._cbs:
            if cb and smach.has_smach_interface(cb):
                self._cb_input_keys.append(cb.get_registered_input_keys())
                self._cb_output_keys.append(cb.get_registered_output_keys())
                self._cb_outcomes.append(cb.get_registered_outcomes())

                self.register_input_keys(self._cb_input_keys[-1])
                self.register_output_keys(self._cb_output_keys[-1])
                self.register_outcomes(self._cb_outcomes[-1])
{% endmacro %}

#
# Macro for rendering 'callbacks' execution in state class execute() methods.
#
{% macro render_execute_callbacks() %}
        # Call callbacks
        for (cb, ik, ok) in zip(self._cbs,
                                self._cb_input_keys,
                                self._cb_output_keys):

            # Call callback with limited userdata
            cb_outcome = cb(smach.Remapper(userdata,ik,ok,{}))
{% endmacro %}

#
# Macro for rendering 'callbacks' dicts in state instantiations.
#
{% macro render_callbacks(name, callbacks, indent=51) %}{{ 'callbacks = ' | indent(indent, true) }}[{% for cb_key, cb_val in callbacks.items() %}{% if cb_val is expression %}'{{ cb_key|lower + '_' + name|lower + '_lambda_cb' }}'{% else %}'{{ cb_val }}'{% endif %}{% if not loop.last %}, {% endif %}{% endfor %}]{% endmacro %}


#
# Macro for rendering 'outcome_map' in state instantiations.
#
{% macro render_outcome_map(outcome_map, indent=35) %}{{ 'outcome_map=' | indent(indent, true) }}{{ '{' }}{% for outcome_map_key, outcome_map_val in outcome_map.items() | sort %}{% if outcome_map_val is mapping %}'{{ outcome_map_key }}': {{ '{' }} {% for outcome_map_sub_key, outcome_map_sub_val in outcome_map_val.items() | sort %}'{{ outcome_map_sub_key }}': '{{ outcome_map_sub_val }}'{% if not loop.last %}, {% endif %}{% endfor %}{{ '}' }}{% else %}'{{ outcome_map_key }}': '{{ outcome_map_val }}'{% endif %}{% if not loop.last %},
{{ '' | indent(indent+13, true) }}{% endif %}{% endfor %}{{ '}' }}{% endmacro %}

#
# Macro for rendering 'transitions' in state machine add() methods.
#
{% macro render_transitions(transitions, indent=23) %}{{ 'transitions=' | indent(indent, true) }}{{ '{' }}{% for outcome, transition in transitions.items() | sort %}'{{ outcome }}':'{{ transition }}'{% if not loop.last %},
{{ '' | indent(indent+13, true) }}{% endif %}{% endfor %}{{ '}' }}{% endmacro %}

#
# Macro for rendering 'remapping' in state machine add() methods.
#
{% macro render_remapping(remapping, indent=23) %}{{ 'remapping=' | indent(indent, true) }}{{ '{' }}{% for state_key, userdata_key in remapping.items() | sort %}'{{ state_key }}':'{{ userdata_key }}'{% if not loop.last %},
{{ '' | indent(indent+11, true) }}{% endif %}{% endfor %}{{ '}' }}{% endmacro %}

#
# Macro for importing a module.
#
{% macro import_module(defined_headers, module) %}
{% if 'import_' + module not in defined_headers %}
import {{ module }}
{% do defined_headers.append('import_' + module) %}
{% endif %}{% endmacro %}

#
# Macro for importing a module and renaming it as something else.
#
{% macro import_module_as(defined_headers, module, new_name) %}
{% if 'import_' + module + '_as_' + new_name not in defined_headers %}
import {{ module }} as {{ new_name }}
{% do defined_headers.append('import_' + module + '_as_' + new_name) %}
{% endif %}{% endmacro %}

#
# Macro for importing a name from a module.
#
{% macro from_import(defined_headers, module, name) %}
{% if 'from_' + module + '_import_' + name not in defined_headers %}
from {{ module }} import {{ name }}
{% do defined_headers.append('from_' + module + '_import_' + name) %}
{% endif %}{% endmacro %}

#
# Macro for importing a name from a module and renaming it as something else.
#
{% macro from_import_as(defined_headers, module, name, new_name) %}
{% if 'from_' + module + '_import_' + name + '_as_' + new_name not in defined_headers %}
from {{ module }} import {{ name }} as {{ new_name }}
{% do defined_headers.append('from_' + module + '_import_' + name + '_as_' + new_name) %}
{% endif %}{% endmacro %}
