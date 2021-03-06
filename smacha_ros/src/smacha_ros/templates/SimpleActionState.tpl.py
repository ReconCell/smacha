{% block meta %}
name: SimpleActionState
description:
  SMACH SimpleActionState for calling actions using an action server.
language: Python
framework: SMACH
type: State
tags: [core]
includes:
  - State
extends: []
variables:
- action_server_namespace:
    description:
      The ROS topic/namespace of the action server, e.g.
      '/robot_1/joint_motion_action'.
    type: str
- action:
    description: The action type.
    type: str
- - goal:
      description: The goal as defined by a .action file.
      type: str
  - goal_slots:
      description: Goal slots as defined by userdata.
      type: list of str
  - goal_cb:
      description: A goal callback.
      type: str
- - result_slots:
      description: Result slots as defined by userdata.
      type: list of str
  - result_cb:
      description: A result callback.
      type: str
- - input_keys:
      description:
        The userdata input keys needed for the request callback.
      type: list of str
- - output_keys:
      description:
        The userdata output keys needed for the response callback.
      type: list of str
- - userdata:
      description:
        The definitions for the userdata keys named in the input_keys and
        output_keys variables.
      type: dict of str
outcomes:
  - succeeded
  - aborted
  - preempted
{% endblock meta %}

{% from "Utils.tpl.py" import from_import, render_goal_slots, render_input_keys, render_result_slots, render_output_keys, render_transitions, render_remapping %}

{% extends "State.tpl.py" %}

{% block imports %}
{{ super() }}
{{ from_import(defined_headers, 'actionlib', '*') }}
{% endblock imports %}

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
