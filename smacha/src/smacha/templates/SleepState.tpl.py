{% block meta %}
name: SleepState
description: SMACH state that pauses state machine execution for a specified length of time.
language: Python
framework: SMACH
type: State
tags: [core]
includes:
  - State
extends: []
variables:
- time:
    description: The length of time to sleep for in milliseconds.
    type: int
input_keys: []
output_keys: []
outcomes:
- succeeded
{% endblock meta %}

{% from "Utils.tpl.py" import import_module, render_transitions %}

{% include "State.tpl.py" %}

{% block imports %}
{{ import_module(defined_headers, 'rospy') }}
{% endblock imports %}

{% block defs %}
{% endblock defs %}

{% block class_defs %}
{% if 'class_SleepState' not in defined_headers %}
class SleepState(smach.State):
    def __init__(self, time, input_keys = [], output_keys = [], callbacks = [], outcomes=['succeeded']):
        smach.State.__init__(self, input_keys=input_keys, output_keys=output_keys, outcomes=outcomes)

        self._time = time

    def execute(self, userdata):

        rospy.sleep(self._time)

        return 'succeeded'
{% do defined_headers.append('class_SleepState') %}{% endif %}
{% endblock class_defs %}

{% block header %}
{% endblock header %}

{% block body %}
smach.{{ parent_type }}.add('{{ name }}',
        {{ '' | indent(23, true) }}SleepState({{ time }}){% if transitions is defined %},
{{ render_transitions(transitions) }}{% endif %})
{% endblock body %}
