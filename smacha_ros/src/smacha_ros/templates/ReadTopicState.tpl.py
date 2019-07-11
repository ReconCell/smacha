{% block meta %}
name: ReadTopicState
description:
  SMACH state that reads data from a ROS topic.
language: Python
framework: SMACH
type: State
tags: [core]
includes: []
extends:
  - WaitForMsgState
variables:
- input_keys:
    description:
      Other than the 'topic' input_key, which specifies the topic to be read,
      remaining input_keys are assumed to be used by the optionally specified
      callback functions.
    type: list of str
- - output_keys:
      description:
        If any of the output_keys ['output', 'msg', 'output_msg', 'msg_output']
        are specified, the read topic data will be written to them.
        Any remaining output_keys are assumed to be used for each optionally
        specified and correspondingly named callback function.
      type: list of str
- - callbacks:
      description:
        Either callback function names or backtick-wrapped lambda functions
        for possible modifications to the topic reading procedure.
      type: dict of str
input_keys:
- topic:
    description:
      The name of the topic from which the data should be read.
    type: str
output_keys:
- - output:
      description:
        The default output key for the topic data.
      type: str
  - msg:
      description:
        A possible output key for the topic data.
      type: str
  - output_msg:
      description:
        A possible output key for the topic data.
      type: str
  - msg_output:
      description:
        A possible output key for the topic data.
      type: str
outcomes:
- succeeded
- aborted
{% endblock meta %}

{% extends "WaitForMsgState.tpl.py" %}

{% from "Utils.tpl.py" import import_module, render_init_callbacks, render_execute_callbacks %}

{% block imports %}
{{ super() }}
{{ import_module(defined_headers, 'rostopic') }}
{{ import_module(defined_headers, 'rospy') }}
{{ import_module(defined_headers, 'roslib') }}
{% endblock imports %}

{% block class_defs %}
{{ super() }}
{% if 'class_ReadTopicState' not in defined_headers %}
class ReadTopicState(smach.State):
    def __init__(self, input_keys=['topic'], output_keys=['output'], callbacks = None):
        smach.State.__init__(self, input_keys=input_keys, output_keys=output_keys, outcomes=['succeeded', 'aborted'])

        {{ render_init_callbacks() }}

    def execute(self, userdata):

        {{ render_execute_callbacks() }}

        # Get topic name from userdata
        try:
            topic = userdata.topic
        except Exception as e:
            rospy.logerr('Topic name must be specified as a userdata input key: {}'.format(repr(e)))
            return 'aborted'

        # Default to rospy.AnyMsg, whereby the message type will be dynamically detected.
        msg_class = rospy.AnyMsg

        # If the message type is specified in userdata, use it instead of rospy.AnyMsg
        if 'msg_type' in self._input_keys:
            try:
                msg_class = roslib.message.get_message_class(userdata.msg_type)
            except Exception as e:
                rospy.logwarn('Failed to load message class from message type specified in userdata, proceeding with message type/class detection.')
                msg_class = rospy.AnyMsg
                pass

        # Try detecting the message type/class if necessary.
        # See: https://schulz-m.github.io/2016/07/18/rospy-subscribe-to-any-msg-type/
        if msg_class is rospy.AnyMsg:
            try:
                with WaitForMsgState(topic, msg_class, latch=True) as wait_for_any_msg:
                    any_msg = wait_for_any_msg.waitForMsg()
                    msg_type = any_msg._connection_header['type']
                    rospy.loginfo('Message type for topic {} detected as {}'.format(topic, msg_type))
                    msg_class = roslib.message.get_message_class(msg_type)
            except Exception as e:
                rospy.logerr('Failed to detect message type/class for topic {}: {}'.format(topic, repr(e)))
                return 'aborted'

        # Set up a WaitForMsgState object
        try:
            wait_for_msg = WaitForMsgState(topic, msg_class, latch=True)
        except Exception as e: 
            rospy.logerr('Failed to set up WaitForMsgState object for topic {}: {}'.format(topic, repr(e)))
            return 'aborted'

        # Wait for message
        try:
            msg = wait_for_msg.waitForMsg()
        except Exception as e: 
            rospy.logwarn('Failed to read message from topic {}: {}'.format(topic, repr(e)))
            return 'aborted'

        # Set msg output key if specified
        for output_key in ['msg', 'output', 'output_msg', 'msg_output']:
            if output_key in self._output_keys:
                setattr(userdata, output_key, msg)

        return 'succeeded'
{% do defined_headers.append('class_ReadTopicState') %}{% endif %}
{% endblock class_defs %}
