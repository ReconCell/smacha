{% block meta %}
name: Base
description: SMACH base template.
language: Python
framework: SMACH
type: Base
tags: [core]
includes: []
extends: []
variables:
- - manifest:
      description: ROS manifest name.
      type: str
- - node_name:
      description: ROS node name for the state machine.
      type: str
- outcomes:
    description: A list of possible outcomes of the state machine.
    type: list
- - userdata:
      description: Definitions for userdata needed by child states.
      type: dict
- - function_name:
      description: A name for the main executable state machine function.
      type: str
input_keys: []
output_keys: []
{% endblock meta %}

{% from "Utils.tpl.py" import import_module, render_outcomes, render_userdata %}

{% set defined_headers = [] %}
{% set local_vars = [] %}

{% block base_header %}
#!/usr/bin/env python
{{ base_header }}
{% endblock base_header %}

{% block imports %}
{{ import_module(defined_headers, 'smach') }}
{{ imports }}
{% endblock imports %}

{% block defs %}
{{ defs }}
{% endblock defs %}

{% block class_defs %}
{{ class_defs }}
{% endblock class_defs %}

{% block cb_defs %}
{{ cb_defs }}
{% endblock cb_defs %}

{% if name is defined %}{% set sm_name = name | lower() %}{% else %}{% set sm_name = 'sm' %}{% endif %}

{% block main_def %}
def {% if function_name is defined %}{{ function_name | lower() }}{% else %}main{% endif %}():
    
    {{ main_def | indent(4) }}
{% endblock main_def %}

{% block body %}
    {{ sm_name }} = smach.StateMachine({{ render_outcomes(outcomes) }})

    {# Container header insertion variable indexed by container state name #}
    {% if name in header %}{{ header[name] | indent(4, true) }}{% endif %}

    {# Render container userdata #}
    {% if userdata is defined %}{{ render_userdata(name | lower(), userdata) | indent(4) }}{% endif %}
    {# Render state userdata #}
    {% if name in header_userdata %}{{ header_userdata[name] | indent(4, true) }}{% endif %}

    with {{ sm_name }}:

        {# Container body insertion variable #}
        {{ body | indent(8) }}
{% endblock body %}

{% block footer %}
        {{ footer | indent(8) }}
{% endblock footer %}

{% block execute %}
    {{ execute | indent(4) }}

    outcome = {{ sm_name }}.execute()
{% endblock execute %}

{% block base_footer %}
    {{ base_footer | indent(4) }}
{% endblock base_footer %}

{% block main %}
if __name__ == '__main__':
{{ '' | indent(4, true) }}{% if function_name is defined %}{{ function_name | lower() }}{% else %}main{% endif %}()
{% endblock main %}
