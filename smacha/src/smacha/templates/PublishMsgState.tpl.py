{% block meta %}
name: PublishMsgState
description: >
  SMACH template that publishes a ROS message passed via a userdata input_key
  to a topic. If the message type is not specified, attempts will be made to
  infer it from the message data.
language: Python
framework: SMACH
type: None
tags: [core]
includes: []
extends:
- State
variables:
- - input_keys:
      description: >
        Other than the 'msg' input_key, which specifies the ROS message to be
        published, remaining input_keys are assumed to be used by the
        optionally specified callback functions.
      type: list of str
- - output_keys:
      description: >
        Other than the 'topic' output_key, which specifies the ROS topic to
        which the message should be published, remaining output_keys are
        assumed to be used by the optionally specified callback functions.
      type: list of str
- - callbacks:
      description:
        Either callback function names or backtick-wrapped lambda functions
        for possible modifications to the message publishing procedure.
      type: dict of str
- action:
    description: >
      The action that the state should perform. The options are 'add', 'remove'
      and 'remove_all', which will add the specified message publication topic,
      remove the specified message publication topic, and remove all currently
      published topics respectively.
    type: str
- - frame_id:
      description:
        The id of a specific tf frame to which the published message should be attached.
      type: str
input_keys:
- msg:
    description: >
      The ROS message to be published.
    type:
      - geometry_msgs/Point
      - geometry_msgs/PointStamped
      - geometry_msgs/Pose
      - geometry_msgs/PoseStamped
      - geometry_msgs/PoseArray
      - geometry_msgs/PointCloud
      - geometry_msgs/PointCloud2
- - msg_type:
      description: >
        The specified type of the ROS message to be published. The options are
        the possible types of the 'msg' input_key specified as a str.
      type: str
output_keys:
- topic:
    description:
      The name of the topic to which the message should be published.
    type: str
{% endblock meta %}

{% from "Utils.tpl.py" import import_module, render_transitions, render_remapping, render_input_keys, render_output_keys, render_init_callbacks, render_execute_callbacks, render_callbacks %}

{% extends "State.tpl.py" %}

{% include "ParsePointStamped.tpl.py" %}
{% include "ParsePoseStamped.tpl.py" %}
{% include "ParsePoseArray.tpl.py" %}
{% include "ParsePointCloud.tpl.py" %}
{% include "ParsePointCloud2.tpl.py" %}
{% include "MsgPublisherObserver.tpl.py" %}

{% block imports %}
{{ super() }}
{{ import_module(defined_headers, 'roslib') }}
{% endblock imports %}

{% block class_defs %}
{{ super() }}
{% if 'class_PublishMsgState' not in defined_headers %}
class PublishMsgState(smach.State):
    def __init__(self, name, msg_pub_observer, action, input_keys = ['msg'], output_keys = ['topic'], callbacks = None):
        smach.State.__init__(self, input_keys=input_keys, output_keys=output_keys, outcomes=['succeeded', 'aborted'])

        # Save the state name
        self._name = name

        # Save the MsgPublisherObserver object reference
        self._msg_pub_observer= msg_pub_observer

        # Save the action
        self._action = action

        # Set up dict of parsing functions for certain message types/classes.
        self._msg_parsers = {"<class 'geometry_msgs.msg._Point.Point'>": parse_pointstamped,
                             "<class 'geometry_msgs.msg._PointStamped.PointStamped'>": parse_pointstamped,
                             "<class 'geometry_msgs.msg._Pose.Pose'>": parse_posestamped,
                             "<class 'geometry_msgs.msg._PoseStamped.PoseStamped'>": parse_posestamped,
                             "<class 'geometry_msgs.msg._PoseArray.PoseArray'>": parse_posearray,
                             "<class 'sensor_msgs.msg._PointCloud.PointCloud'>": parse_pointcloud,
                             "<class 'sensor_msgs.msg._PointCloud2.PointCloud2'>": parse_pointcloud2}

        {{ render_init_callbacks() }}

    def _parse_msg(self, msg, msg_type=None):
        # First try using a known parser for a specified msg_type.
        try:
            assert msg_type
            msg_class = str(roslib.message.get_message_class(msg_type))
            published_msg = self._msg_parsers[msg_class](msg)
            return published_msg
        except:
            pass

        # Next, try to select a known parser by checking the type of message.
        try:
            msg_class = str(type(msg))
            published_msg = self._msg_parsers[msg_class](msg)
            return published_msg
        except:
            pass

        # Next, try each message type parser in succession and see if something sticks.
        for _, parser in self._msg_parsers.items():
            try:
                published_msg = parser(msg)
                return published_msg
            except:
                pass

        # Finally, if none of the above stuck, just return the original message.
        return msg

    def execute(self, userdata):

        {{ render_execute_callbacks() }}

        # Parse msg
        if self._action != 'remove_all':
            try:
                if 'msg_type' in self._input_keys:
                    published_msg = self._parse_msg(userdata.msg, msg_type=userdata.msg_type)
                else:
                    published_msg = self._parse_msg(userdata.msg)
            except Exception as e:
                rospy.logerr('Failed to parse message: '.format(repr(e)))
                return 'aborted'

        # Get topic if it's specified as an input key
        if 'topic' in self._input_keys:
            topic = userdata.topic
        # Otherwise, construct it from the state name
        else:
            topic = 'smacha/' + self._name.lower()

        # Add or remove the message + publisher
        outcome = 'aborted'
        if self._action == 'add':
            if 'frame_id' in self._input_keys:
                outcome = self._msg_pub_observer.add(published_msg, topic, frame_id=userdata.frame_id)
            else:
                outcome = self._msg_pub_observer.add(published_msg, topic)
        elif self._action == 'remove':
            outcome = self._msg_pub_observer.remove(topic)
        elif self._action == 'remove_all':
            outcome = self._msg_pub_observer.remove_all()

        # Set topic output key if specified
        if self._action == 'add' and outcome == 'succeeded':
            for output_key in ['topic', 'output_topic', 'topic_output']:
                if output_key in self._output_keys:
                    setattr(userdata, output_key, topic)

        # Set msg output key if specified
        if self._action == 'add' and outcome == 'succeeded':
            for output_key in ['msg', 'output_msg', 'msg_output']:
                if output_key in self._output_keys:
                    setattr(userdata, output_key, published_msg)

        return outcome
{% do defined_headers.append('class_PublishMsgState') %}{% endif %}
{% endblock class_defs %}

{% block main_def %}
{{ super() }}
{% if 'tf_msg_pub_observer' not in defined_headers %}
tf_msg_pub_observer = MsgPublisherObserver(sub_topic='/tf')
{% do defined_headers.append('tf_msg_pub_observer') %}{% endif %}
{% endblock main_def %}

{% block body %}
smach.{{ parent_type }}.add('{{ name }}',
        {{ '' | indent(23, true) }}PublishMsgState('{{ name }}', tf_msg_pub_observer, '{{ action }}'{% if input_keys is defined %}, {{ render_input_keys(input_keys, indent=0) }}{% endif %}{% if output_keys is defined %}, {{ render_output_keys(output_keys, indent=0) }}{% endif %}{% if callbacks is defined %}, {{ render_callbacks(name, uuid, callbacks, indent=0) }}{% endif %}){% if transitions is defined %},
{{ render_transitions(transitions) }}{% endif %}{% if remapping is defined %},
{{ render_remapping(remapping) }}{% endif %})
{% endblock body %}
