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
      The action that the state should perform. The options are 'start', 'stop'
      and 'stop_all', which will start publication of the specified message to
      the specified topic, stop publication to the specified topic, and stop
      publication of all currently published topics respectively.
    type: str
input_keys:
- msg:
    description: >
      The ROS message to be published.

      In case the intended content of the message is specified in list form,
      parsers will be applied for known message types in an attempt to convert
      the list input into a corresponding ROS message type.
      Parsers for the following message types are currently operational:
      geometry_msgs/PointStamped,
      geometry_msgs/PoseStamped,
      geometry_msgs/PoseArray,
      geometry_msgs/PointCloud, and
      geometry_msgs/PointCloud2.

      E.g. if msg is specified as [1.0, 2.0, 3.0], then it should be parsed
      and converted to a geometry_msgs/PointStamped type, whereas if it is
      specified as [[1.0, 2.0, 3.0], [0.0, 0.0, 0.0, 1.0]], then it should be
      parsed and converted to a geometry_msgs/PoseStamped type.
    type:
      - genpy.message.Message
      - list
- topic:
    description:
      The name of the topic to which the message should be published.
    type: str
- - msg_type:
      description: >
        The specified type of the ROS message to be published. The options are
        the possible types of the 'msg' input_key specified as a str.
      type: str
- - frame_id:
      description:
        The id of a specific tf frame to which the published message should be attached.
      type: str
- - rate:
      description:
        The rate at which the message should be published specified in hertz (hz).
      type: float
output_keys:
- msg:
    description: >
      See input_keys_description.
    type:
      - genpy.message.Message
      - list
- topic:
    description:
      See input_keys description.
    type: str
- - rate:
      description:
        The rate at which the message should be published specified in hertz (hz).
      type: float
{% endblock meta %}

{% from "Utils.tpl.py" import import_module, render_transitions, render_remapping, render_input_keys, render_output_keys, render_init_callbacks, render_execute_callbacks, render_callbacks %}

{% extends "State.tpl.py" %}

{% include "ParsePointStamped.tpl.py" %}
{% include "ParsePoseStamped.tpl.py" %}
{% include "ParsePoseArray.tpl.py" %}
{% include "ParsePointCloud.tpl.py" %}
{% include "ParsePointCloud2.tpl.py" %}
{% include "MsgPublisher.tpl.py" %}

{% block imports %}
{{ super() }}
{{ import_module(defined_headers, 'roslib') }}
{% endblock imports %}

{% block class_defs %}
{{ super() }}
{% if 'class_PublishMsgState' not in defined_headers %}
class PublishMsgState(smach.State):
    def __init__(self, name, msg_publisher, action, input_keys = ['msg', 'topic', 'rate'], output_keys = ['msg', 'topic'], callbacks = None):
        smach.State.__init__(self, input_keys=input_keys, output_keys=output_keys, outcomes=['succeeded', 'aborted'])

        # Save the state name
        self._name = name

        # Save the MsgPublisherObserver object reference
        self._msg_publisher = msg_publisher

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

        # Start or stop the message publisher
        outcome = 'aborted'
        if self._action == 'start':
            # Parse msg
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

            # Get rate if it's specified as an input key
            if 'rate' in self._input_keys:
                rate = userdata.rate
            else:
                rate = 100.0

            # Get callback if it's specified as an input key
            if 'callback' in self._input_keys:
                callback = userdata.callback
            else:
                callback = ''

            # Get frame_id if it's specified as an input key
            if 'frame_id' in self._input_keys:
                frame_id = userdata.frame_id
            else:
                frame_id = ''

            # Start the publisher
            outcome = self._msg_publisher.start(published_msg, topic, rate, frame_id=frame_id, callback=callback)

        elif self._action == 'stop':
            outcome = self._msg_publisher.stop(topic)

        elif self._action == 'stop_all':
            outcome = self._msg_publisher.stop_all()

        # Set topic output key if specified
        if self._action == 'start' and outcome == 'succeeded':
            for output_key in ['topic', 'output_topic', 'topic_output']:
                if output_key in self._output_keys:
                    setattr(userdata, output_key, topic)

        # Set msg output key if specified
        if self._action == 'start' and outcome == 'succeeded':
            for output_key in ['msg', 'output_msg', 'msg_output']:
                if output_key in self._output_keys:
                    setattr(userdata, output_key, published_msg)

        return outcome
{% do defined_headers.append('class_PublishMsgState') %}{% endif %}
{% endblock class_defs %}

{% block main_def %}
{{ super() }}
{% if 'msg_publisher' not in defined_headers %}
msg_publisher = MsgPublisher()
{% do defined_headers.append('msg_publisher') %}{% endif %}
{% endblock main_def %}

{% block header %}
{{ super() }}
{#
 # By using this bit of trickery, we ensure that the mandatory userdata variables
 # 'rate' and 'topic' get defined as an empty string before any other userdata
 # variables are defined (these are rendered via the 'header_userdata' block
 # usually, which is rendered after the 'header' block). This allows for the
 # specification of 'rate' and 'topic' to be omitted in the SMACHA script state
 # definitions to simplify scripts under certain circumstances,
 # e.g. 'topic' can be omitted when the 'action' variable is set to 'stop_all'.
 #}
{% if mandatory_userdata is not defined %}{% set mandatory_userdata = dict() %}{% endif %}
{% if 'rate' not in mandatory_userdata.keys() %}{% set _dummy = mandatory_userdata.update({'rate':100.0}) %}{% endif %}
{% if action == 'stop_all' and 'topic' not in mandatory_userdata.keys() %}{% set _dummy = mandatory_userdata.update({'topic':''}) %}{% endif %}
{% if mandatory_userdata is defined %}{{ render_userdata(parent_sm_name, mandatory_userdata) }}{% endif %}
{% endblock header %}

{% block body %}
smach.{{ parent_type }}.add('{{ name }}',
        {{ '' | indent(23, true) }}{{ class_name }}('{{ name }}', msg_publisher, '{{ action }}'{% if input_keys is defined %}, {{ render_input_keys(input_keys, indent=0) }}{% endif %}{% if output_keys is defined %}, {{ render_output_keys(output_keys, indent=0) }}{% endif %}{% if callbacks is defined %}, {{ render_callbacks(name, uuid, callbacks, indent=0) }}{% endif %}){% if transitions is defined %},
{{ render_transitions(transitions) }}{% endif %}{% if remapping is defined %},
{{ render_remapping(remapping) }}{% endif %})
{% endblock body %}
