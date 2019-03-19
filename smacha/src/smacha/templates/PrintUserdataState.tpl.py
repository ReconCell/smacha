{% block meta %}
name: PrintUserdataState
description: SMACH state that prints the values of userdata input_keys to standard output.
language: Python
framework: SMACH
type: State
tags: [core]
includes:
  - State
extends: []
variables: []
input_keys: []
output_keys: []
outcomes:
- succeeded
{% endblock meta %}

{% from "Utils.tpl.py" import import_module, render_input_keys, render_transitions, render_remapping %}

{% include "State.tpl.py" %}

{% block imports %}
{{ import_module(defined_headers, 'rospy') }}
{% endblock imports %}

{% block defs %}
{% endblock defs %}

{% block class_defs %}
{% if 'class_PrintUserdataState' not in defined_headers %}
class PrintUserdataState(smach.State):
    def __init__(self, input_keys = [], output_keys = [], callbacks = [], outcomes=['succeeded']):
        smach.State.__init__(self, input_keys=input_keys, output_keys=output_keys, outcomes=outcomes)

    def execute(self, userdata):

        # Print input keys to terminal
        for input_key in self._input_keys:
            rospy.loginfo('userdata.{}: {}'.format(input_key, userdata[input_key]))

        return 'succeeded'
{% do defined_headers.append('class_PrintUserdataState') %}{% endif %}
{% endblock class_defs %}

{% block header %}
{% endblock header %}

{% block body %}
smach.{{ parent_type }}.add('{{ name }}',
        {{ '' | indent(23, true) }}PrintUserdataState({% if input_keys is defined %}{{ render_input_keys(input_keys, indent=0) }}{% endif %}){% if transitions is defined %},
{{ render_transitions(transitions) }}{% endif %}{% if remapping is defined %},
{{ render_remapping(remapping) }}{% endif %})
{% endblock body %}
