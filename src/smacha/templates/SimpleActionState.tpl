{% from "Utils.tpl" import render_goal_slots, render_input_keys, render_result_slots, render_output_keys, render_transitions, render_remapping %}

{% include "State.tpl" %}

{% block base_header %}
{% endblock base_header %}

{% block imports %}
{% if 'actionlib_import' not in defined_headers %}
from actionlib import *
{% do defined_headers.append('actionlib_import') %}
{% endif %}
{% endblock imports %}

{% block defs %}
{% endblock defs %}

{% block class_defs %}
{% endblock class_defs %}

{% block header %}
{% endblock header %}

{% block main_def %}
{% endblock main_def %}

{% block body %}
smach.{{ parent_type }}.add('{{ name }}',
{{ '' | indent(23, true) }}smach_ros.SimpleActionState('{{ action_server_namespace }}', {{ action }}{% if goal is defined %},
{{ 'goal = ' | indent(51, true) }}{{ goal }}{% elif goal_slots is defined %},
{{ render_goal_slots(goal_slots) }}{% elif goal_cb is defined %},
{{ 'goal_cb = ' | indent(51, true) }}{{ goal_cb }}{% if input_keys is defined %},
{{ render_input_keys(input_keys) }}{% endif %}{% endif %}{% if result_slots is defined %},
{{ render_result_slots(result_slots) }}{% elif result_cb is defined %},
{{ 'result_cb = ' | indent(51, true) }}{{ result_cb }}{% if output_keys is defined %},
{{ render_output_keys(output_keys) }}{% endif %}{% endif %}){% if transitions is defined %},
{{ render_transitions(transitions) }}{% endif %}{% if remapping is defined %},
{{ render_remapping(remapping) }}{% endif %})
{% endblock body %}

{% block footer %}
{% endblock footer %}

{% block execute %}
{% endblock execute %}

{% block spin %}
{% endblock spin %}

{% block base_footer %}
{% endblock base_footer %}

{% block main %}
{% endblock main %}
