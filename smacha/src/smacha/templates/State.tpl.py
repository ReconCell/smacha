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

{% from "Utils.tpl.py" import render_def_lambda_callbacks, render_userdata %}

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
