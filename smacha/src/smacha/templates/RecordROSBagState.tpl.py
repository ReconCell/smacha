{% from "Utils.tpl.py" import import_module, render_init_callbacks, render_execute_callbacks %}

{% extends "State.tpl.py" %}

{% include "ROSBagRecorderObserver.tpl.py" %}

{% block class_defs %}
{{ super() }}
{% if 'class_RecordROSBagState' not in defined_headers %}
class RecordROSBagState(smach.State):
    def __init__(self, name, bag_rec_observer, action, input_keys=['file', 'topics'], output_keys=[], callbacks = None):
        smach.State.__init__(self, input_keys=input_keys, output_keys=output_keys, outcomes=['succeeded', 'aborted'])

        # Save the state name
        self._name = name

        # Save the ROSBagRecorderObserver object reference
        self._bag_rec_observer= bag_rec_observer

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
            outcome = self._bag_rec_observer.start(bag_file, topics)
        elif self._action == 'stop':
            outcome = self._bag_rec_observer.stop(bag_file)
        elif self._action == 'stop_all':
            outcome = self._bag_rec_observer.stop_all()

        return outcome
{% do defined_headers.append('class_RecordROSBagState') %}{% endif %}
{% endblock class_defs %}

{% block main_def %}
{{ super() }}
{% if 'bag_rec_observer' not in defined_headers %}
bag_rec_observer = ROSBagRecorderObserver()
{% do defined_headers.append('bag_rec_observer') %}{% endif %}
{% endblock main_def %}

{% block body %}
smach.{{ parent_type }}.add('{{ name }}',
        {{ '' | indent(23, true) }}{{ class_name }}('{{ name }}', bag_rec_observer, '{{ action }}'{% if input_keys is defined %}, {{ render_input_keys(input_keys, indent=0) }}{% endif %}{% if output_keys is defined %}, {{ render_output_keys(output_keys, indent=0) }}{% endif %}{% if callbacks is defined %}, {{ render_callbacks(name, uuid, callbacks, indent=0) }}{% endif %}){% if transitions is defined %},
{{ render_transitions(transitions) }}{% endif %}{% if remapping is defined %},
{{ render_remapping(remapping) }}{% endif %})
{% endblock body %}
