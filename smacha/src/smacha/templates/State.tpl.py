{% block meta %}
name: StateMachine
description: SMACH template containing code common to all state templates.
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

{% from "Utils.tpl.py" import render_input_keys, render_output_keys, render_outcomes, render_callbacks, render_transitions, render_remapping, render_def_lambda_callbacks, render_userdata %}

{% block base_header %}
{% endblock base_header %}

{% block imports %}
{% endblock imports %}

{% block defs %}
{% endblock defs %}

{% block class_defs %}
{% endblock class_defs %}

{% block cb_defs %}
{% if callbacks is defined %}
{% if input_keys is defined %}
{{ render_def_lambda_callbacks(defined_headers, class_name, name, uuid, input_keys, callbacks) }}
{% else %}
{{ render_def_lambda_callbacks(defined_headers, class_name, name, uuid, [], callbacks) }}
{% endif %}
{% endif %}
{% endblock cb_defs %}

{% block main_def %}
{% endblock main_def %}

{% block header %}
{% if userdata is defined %}{{ render_userdata(parent_sm_name, userdata) }}{% endif %}
{% endblock header %}

{% block body %}
smach.{{ parent_type }}.add('{{ name }}',
        {{ '' | indent(23, true) }}{{ class_name }}({% if input_keys is defined %}{{ render_input_keys(input_keys, indent=0) }}{% endif %}{% if output_keys is defined %}{% if input_keys is defined %}, {% endif %}{{ render_output_keys(output_keys, indent=0) }}{% endif %}{% if callbacks is defined %}{% if input_keys is defined or output_keys is defined %}, {% endif %}{{ render_callbacks(name, uuid, callbacks, indent=0) }}{% endif %}{% if outcomes is defined %}{% if input_keys is defined or output_keys is defined or callbacks is defined %}, {% endif %}{{ render_outcomes(outcomes, indent=0) }}{% endif %}){% if transitions is defined %},
{{ render_transitions(transitions) }}{% endif %}{% if remapping is defined %},
{{ render_remapping(remapping) }}{% endif %})
{% endblock body %}

{% block footer %}
{% endblock footer %}

{% block introspection_server %}
{% endblock introspection_server %}

{% block execute %}
{% endblock execute %}

{% block spin %}
{% endblock spin %}

{% block base_footer %}
{% endblock base_footer %}

{% block main %}
{% endblock main %}
