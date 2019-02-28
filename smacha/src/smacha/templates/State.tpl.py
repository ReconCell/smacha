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

{% from "Utils.tpl.py" import render_userdata %}

{% set local_vars = [] %}

{% block header %}
{% if userdata is defined %}{{ render_userdata(parent_sm_name, userdata) }}{% endif %}
{% endblock header %}
