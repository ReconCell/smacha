{% block meta %}
name: StateMachine
description: SMACH StateMachine container for nesting child states in a parent state.
language: Python
framework: SMACH
type: Container
tags: [core]
includes: []
extends: []
variables:
- outcomes:
    description: The possible StateMachine container state outcomes.
    type: list of str
- - input_keys:
      description: The names of the userdata input keys needed by the state.
      type: list of str
- - output_keys:
      description: The names of the userdata output keys produced by the state.
      type: list of str
- - userdata:
      description: The definitions for the userdata keys named in the input_keys and output_keys variables.
      type: dict of str
{% endblock meta %}

{% from "Utils.tpl.py" import render_outcomes, render_input_keys, render_output_keys, render_userdata, render_transitions, render_remapping %}

{% set sm_name = ['sm_', name | lower()] | join() %}

{% block upper_comments %}
#----------------------------------------------------------------------------------------
# BEGIN: {{ name }}
# TEMPLATE: StateMachine
#
{% endblock upper_comments %}

{% block body %}
{{ sm_name }} = smach.StateMachine({{ render_outcomes(outcomes) }}{% if input_keys is defined %},
{{ render_input_keys(input_keys) }}{% endif %}{% if output_keys is defined %},
{{ render_output_keys(output_keys) }}{% endif %})

{% if userdata is defined %}{{ render_userdata('sm_' + (name | lower()), userdata) }}{% endif %}
{% if name in header %}{{ header[name] }}{% endif %}

with {{ sm_name }}:

    {{ body | indent(4) }}

smach.{{ parent_type }}.add('{{ name }}', {{ sm_name }}{% if transitions is defined and parent_type != 'Concurrence' %},
{{ render_transitions(transitions) }}{% endif %}{% if remapping is defined %},
{{ render_remapping(remapping) }}{% endif %})
{% endblock body %}

{% block lower_comments %}
#
# END: {{ name }}
#----------------------------------------------------------------------------------------
{% endblock lower_comments %}
