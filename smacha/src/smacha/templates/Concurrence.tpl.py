{% block meta %}
name: Concurrence
description: SMACH Concurrence container for running states in parallel.
language: Python
framework: SMACH
type: Container
tags: [core]
includes: []
extends: []
variables: []
input_keys: []
output_keys: []
outcomes: []
{% endblock meta %}

{% from "Utils.tpl.py" import render_outcomes, render_outcome_map, render_input_keys, render_output_keys, render_userdata, render_transitions, render_remapping %}

{% set sm_name = ['sm_', name | lower()] | join() %}

{% block upper_comments %}
#----------------------------------------------------------------------------------------
# BEGIN: {{ name }}
# TEMPLATE: Concurrence
#
{% endblock upper_comments %}

{% block body %}
{{ sm_name }} = smach.Concurrence({{ render_outcomes(outcomes) }},
{{ 'default_outcome=' | indent(35, true) }}'{{ default_outcome }}',
{{ render_outcome_map(outcome_map) }}{% if input_keys is defined %},
{{ render_input_keys(input_keys, indent=35) }}{% endif %}{% if output_keys is defined %},
{{ render_output_keys(output_keys) }}{% endif %})

{# Container header insertion variable indexed by container state name #}
{% if name in header %}{{ header[name] }}{% endif %}

{# Render container userdata #}
{% if userdata is defined %}{{ render_userdata(name | lower(), userdata) }}{% endif %}
{# Render state userdat #}
{% if name in header_userdata %}{{ header_userdata[name] | indent(4, true) }}{% endif %}

with {{ sm_name }}:

    {# Container body insertion variable #}
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
