{% from "Utils.tpl" import render_outcomes, render_input_keys, render_output_keys, render_userdata, render_transitions, render_remapping %}

{% block meta %}
template:
  name: StateMachine
  description: SMACH StateMachine Container 
  language: Python
variables:
  name:
    description: State name.
    type: str
  sm_name:
    description: State variable name.
    type: str
{% endblock meta %}

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
