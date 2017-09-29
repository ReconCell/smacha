{% from "Utils.tpl" import render_transitions %}

{% include "State.tpl" %}

{% block imports %}
{% if 'import_tf2_ros' not in defined_headers %}
import tf2_ros
{% do defined_headers.append('import_tf2_ros') %}
{% endif %}
{% endblock imports %}

{% block class_defs %}
{% if 'class_TF2ListenerState' not in defined_headers %}
class TF2ListenerState(smach.State):
    def __init__(self, target_frame, source_frame, output_key, time=rospy.Time(0), duration=rospy.Duration(1)):

        self._target_frame = target_frame
        self._source_frame = source_frame
        self._output_key = output_key
        self._time = time
        self._duration = duration

        smach.State.__init__(self,
                             outcomes=[{% for outcome, transition in transitions.items() %}'{{ outcome }}'{% if not loop.last %}, {% endif %}{% endfor %}],
                             input_keys=[],
                             output_keys=[self._output_key])

        self.tf2_buffer = tf2_ros.Buffer()
        self.tf2_listener = tf2_ros.TransformListener(self.tf2_buffer)
        
        while not (self.tf2_buffer.can_transform(self._target_frame, self._source_frame, self._time)):
          pass

    def execute(self, userdata):
        rospy.loginfo('Executing TFListenerState for target_frame ' + self._target_frame + ' and source_frame ' + self._source_frame)
        while not (self.tf2_buffer.can_transform(self._target_frame, self._source_frame, self._time)):
            pass
        setattr(userdata, self._output_key, self.tf_listener.lookupTransform(self._target_frame, self._source_frame, self._time))
        return {% if successful_outcome is defined %}'{{ successful_outcome }}{% else %}'succeeded'{% endif %}
{% do defined_headers.append('class_TF2ListenerState') %}
{% endif %}
{% endblock class_defs %}

{% block header %}
{% endblock header %}

{% block body %}
smach.StateMachine.add('READ_HEXAPOD_CURRENT_POSE', TF2ListenerState('{{ target_frame }}', '{{ source_frame }}', '{{ output_key }}'){% if transitions is defined %},
{{ render_transitions(transitions) }}{% endif %},
{{ 'remapping=' | indent(23, true) }}{'{{ output_key }}':'{{ output_key }}'})
{% endblock body %}