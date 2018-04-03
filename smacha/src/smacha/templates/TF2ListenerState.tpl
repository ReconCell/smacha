{% block meta %}
name: TF2ListenerState
description: SMACH state for listening for and returning a particular transform on TF2.
language: Python
framework: SMACH
type: State
tags: [core]
includes:
  - State
extends: []
variables: []
input_keys:
- target:
    description: The name of the target reference frame.
    type: str
- source:
    description: The name of the source reference frame.
    type: str
output_keys:
- output:
    description: The transform.
    type: geometry_msgs/TransformStamped
outcomes:
- succeeded
- aborted
{% endblock meta %}

{% from "Utils.tpl" import import_module, render_transitions, render_remapping %}

{% include "State.tpl" %}
{% include "TF2ListenerSingleton.tpl" %}

{% block base_header %}
{% endblock base_header %}

{% block imports %}
{{ import_module(defined_headers, 'tf2_ros') }}
{% endblock imports %}

{% block defs %}
{% endblock defs %}

{% block class_defs %}
{% if 'class_TF2ListenerState' not in defined_headers %}
class TF2ListenerState(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'aborted'],
                             input_keys=['target', 'source'],
                             output_keys=['output'])

        self._tf2_buffer, self._tf2_listener = TF2ListenerSingleton.get()
        
    def execute(self, userdata):
        # Get target from userdata
        if 'target' in userdata:
          target = userdata.target
        else:
          raise ValueError('target should be in userdata!')
        
        # Get source from userdata
        if 'source' in userdata:
          source = userdata.source
        else:
          raise ValueError('source should be in userdata!')

        try:
          # Get/set current time/duration
          time=rospy.Time(0)
          duration=rospy.Duration(1)

          rospy.loginfo('Executing TFListenerState for target ' + target + ' and source ' + source)
          while not (self._tf2_buffer.can_transform(target, source, time)):
              pass
          setattr(userdata, 'output', self._tf2_buffer.lookup_transform(target, source, time))
        except Exception as e:
          rospy.logerr('Error when reading transform for target ' + target + ' and source ' + source + ' from TF2!')
          return 'aborted'

        return 'succeeded'
{% do defined_headers.append('class_TF2ListenerState') %}{% endif %}
{% endblock class_defs %}

{% block header %}
{% endblock header %}

{% block main_def %}
{% endblock main_def %}

{% block body %}
smach.StateMachine.add('{{ name }}', TF2ListenerState(){% if transitions is defined %},
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
