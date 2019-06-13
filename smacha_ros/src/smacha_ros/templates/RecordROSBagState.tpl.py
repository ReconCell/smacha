{% block meta %}
name: ROSBagAPIThreadRecorder
description:
  SMACH template that can start and stop rosbag recordings using different types
  of recorder classes, namely ROSBagCLIProcessRecorder or ROSBagAPIThreadRecorder.

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

{% from "Utils.tpl.py" import import_module, render_init_callbacks, render_execute_callbacks %}

{% extends "State.tpl.py" %}

{% if recorder is not defined %}
{% set recorder = 'ROSBagCLIProcessRecorder' %}
{% include "ROSBagCLIProcessRecorder.tpl.py" %}
{% elif recorder == 'ROSBagAPIThreadRecorder' %}
{% include "ROSBagAPIThreadRecorder.tpl.py" %}
{% else %}
{% include "ROSBagCLIProcessRecorder.tpl.py" %}
{% endif %}

{% block class_defs %}
{{ super() }}
{% if 'class_RecordROSBagState' not in defined_headers %}
class RecordROSBagState(smach.State):
    def __init__(self, name, bag_recorder, action, input_keys=['file', 'topics'], output_keys=[], callbacks = None):
        smach.State.__init__(self, input_keys=input_keys, output_keys=output_keys, outcomes=['succeeded', 'aborted'])

        # Save the state name
        self._name = name

        # Save the ROSBagRecorder object reference
        self._bag_recorder= bag_recorder

        # Save the action
        self._action = action

        {{ render_init_callbacks() }}

    def execute(self, userdata):

        {{ render_execute_callbacks() }}

        # Get filename from userdata
        try:
            bag_file = userdata.file
            assert(isinstance(bag_file, str))
        except Exception as e:
            rospy.logerr('The rosbag filename must be specified as a userdata input key: {}'.format(repr(e)))
            return 'aborted'

        # Get topic names from userdata
        try:
            topics = userdata.topics
            assert(not any(not isinstance(x, str) for x in topics))
        except Exception as e:
            rospy.logerr('Topic names must be specified as a userdata input key: {}'.format(repr(e)))
            return 'aborted'

        # Start or stop recording
        outcome = 'aborted'
        if self._action == 'start' or self._action == 'record':
            outcome = self._bag_recorder.start(bag_file, topics)
        elif self._action == 'stop':
            outcome = self._bag_recorder.stop(bag_file)
        elif self._action == 'stop_all':
            outcome = self._bag_recorder.stop_all()

        return outcome
{% do defined_headers.append('class_RecordROSBagState') %}{% endif %}
{% endblock class_defs %}

{% block main_def %}
{{ super() }}
{% if 'bag_recorder' not in defined_headers %}
bag_recorder = {{ recorder }}()
{% do defined_headers.append('bag_recorder') %}{% endif %}
{% endblock main_def %}

{% block header %}
{{ super() }}
{#
 # By using this bit of trickery, we ensure that the mandatory userdata variables
 # 'file' and 'topics' get defined as empty strings before any other userdata
 # variables are defined (these are rendered via the 'header_userdata' block
 # usually, which is rendered after the 'header' block). This allows for the
 # specification of 'file' and 'topics' to be omitted in the SMACHA script state
 # definitions when the 'action' variable is set to 'stop_all', for example.
 #}
{% if mandatory_userdata is not defined %}{% set mandatory_userdata = dict() %}{% endif %}
{% if action == 'stop_all' and 'file' not in mandatory_userdata.keys() %}{% set _dummy = mandatory_userdata.update({'file':''}) %}{% endif %}
{% if (action == 'stop_all' or action == 'stop') and 'topics' not in mandatory_userdata.keys() %}{% set _dummy = mandatory_userdata.update({'topics':''}) %}{% endif %}
{% if mandatory_userdata is defined %}{{ render_userdata(parent_sm_name, mandatory_userdata) }}{% endif %}
{% endblock header %}

{% block body %}
smach.{{ parent_type }}.add('{{ name }}',
        {{ '' | indent(23, true) }}{{ class_name }}('{{ name }}', bag_recorder, '{{ action }}'{% if input_keys is defined %}, {{ render_input_keys(input_keys, indent=0) }}{% endif %}{% if output_keys is defined %}, {{ render_output_keys(output_keys, indent=0) }}{% endif %}{% if callbacks is defined %}, {{ render_callbacks(name, uuid, callbacks, indent=0) }}{% endif %}){% if transitions is defined %},
{{ render_transitions(transitions) }}{% endif %}{% if remapping is defined %},
{{ render_remapping(remapping) }}{% endif %})
{% endblock body %}
