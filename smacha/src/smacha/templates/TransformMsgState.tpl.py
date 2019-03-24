{% block meta %}
name: TransformMsgState
description: >
  SMACH template that transforms a message from one tf frame to another.
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
        Other than the input_keys specified below, remaining input_keys are
        assumed to be used by the optionally specified callback functions.
      type: list of str
- - output_keys:
      description: >
        Other than the 'output_msg' output_key, which specifies the output_key
        name of the transformed message, remaining output_keys are assumed to
        be used by the optionally specified callback functions.
      type: list of str
- - callbacks:
      description:
        Either callback function names or backtick-wrapped lambda functions
        for possible modifications to the message transformation procedure.
      type: dict of str
- - transform_type:
      description: >
        The type of transform to be applied. Options are 'transform',
        'translation' or 'rotation'.  Default: 'transform'.
input_keys:
- msg:
    description: >
      The ROS message to be transformed.
    type:
      - geometry_msgs/Point
      - geometry_msgs/PointStamped
      - geometry_msgs/Pose
      - geometry_msgs/PoseStamped
      - geometry_msgs/PoseArray
      - geometry_msgs/PointCloud
      - geometry_msgs/PointCloud2
- - source:
      description: >
        The id of a tf frame from which the specified ROS message should be transformed.
        If 'source' is not specified, an attempt will be made to infer it from the 'msg'
        header.
      type: str
- - frame_id:
      description:
        The id of a tf frame to which the specified ROS message should be transformed.
      type: str
  - target:
      description:
        The id of a tf frame to which the specified ROS message should be transformed.
      type: str
output_keys:
- - output_msg:
      description: >
        The default output_key for the transformed ROS message.
      type:
        - geometry_msgs/Point
        - geometry_msgs/PointStamped
        - geometry_msgs/Pose
        - geometry_msgs/PoseStamped
        - geometry_msgs/PoseArray
        - geometry_msgs/PointCloud
        - geometry_msgs/PointCloud2
  - msg:
      description: >
        A possible output_key for the transformed ROS message matching the
        'msg' input_key, ensuring that the 'msg' input_key itself is
        transformed.
      type:
        - geometry_msgs/Point
        - geometry_msgs/PointStamped
        - geometry_msgs/Pose
        - geometry_msgs/PoseStamped
        - geometry_msgs/PoseArray
        - geometry_msgs/PointCloud
        - geometry_msgs/PointCloud2
  - output_msg:
      description: >
        A possible output_key for the transformed ROS message.
      type:
        - geometry_msgs/Point
        - geometry_msgs/PointStamped
        - geometry_msgs/Pose
        - geometry_msgs/PoseStamped
        - geometry_msgs/PoseArray
        - geometry_msgs/PointCloud
        - geometry_msgs/PointCloud2
- - transform:
      description: >
        The default output_key for the transform.
      type:
        - geometry_msgs/TransformStamped
  - output_transform:
      description: >
        A possible output_key for the transform.
      type:
        - geometry_msgs/TransformStamped
  - transform_output:
      description: >
        A possible output_key for the transform.
      type:
        - geometry_msgs/TransformStamped
outcomes:
- succeeded
- aborted
{% endblock meta %}

{% from "Utils.tpl.py" import import_module, import_module_as, from_import, from_import_as, render_transitions, render_remapping, render_input_keys, render_output_keys,  render_init_callbacks, render_execute_callbacks, render_callbacks %}

{% extends "State.tpl.py" %}

{% include "ParseTransformStamped.tpl.py" %}
{% include "ParsePointStamped.tpl.py" %}
{% include "ParsePose.tpl.py" %}
{% include "ParsePoseStamped.tpl.py" %}
{% include "ParsePoseArray.tpl.py" %}
{% include "ParsePointCloud.tpl.py" %}
{% include "ParsePointCloud2.tpl.py" %}
{% include "TF2ListenerSingleton.tpl.py" %}

{% block imports %}
{{ super() }}
{{ import_module_as(defined_headers, 'numpy', 'np') }}
{{ from_import(defined_headers, 'tf2_geometry_msgs', 'do_transform_point') }}
{{ from_import(defined_headers, 'tf2_geometry_msgs', 'do_transform_pose') }}
{{ from_import(defined_headers, 'tf2_sensor_msgs', 'do_transform_cloud') }}
{% endblock imports %}

{% block class_defs %}
{{ super() }}
{% if 'class_TransformMsgState' not in defined_headers %}
class TransformMsgState(smach.State):
    def __init__(self, transform_type='transform',
                       input_keys=['msg', 'frame_id'], output_keys=['output_msg'], callbacks=None):
        smach.State.__init__(self, input_keys=input_keys, output_keys=output_keys, outcomes=['succeeded', 'aborted'])

        # Save transform type
        self._transform_type = transform_type

        # Get references to the TF2ListenerSingleton buffer and listener objects
        self._tf2_buffer, self._tf2_listener = TF2ListenerSingleton.get()

        # Set up dict of parsing functions for certain message types/classes.
        self._msg_parsers = {"<class 'geometry_msgs.msg._Point.Point'>": parse_pointstamped,
                             "<class 'geometry_msgs.msg._PointStamped.PointStamped'>": parse_pointstamped,
                             "<class 'geometry_msgs.msg._Pose.Pose'>": parse_pose,
                             "<class 'geometry_msgs.msg._PoseStamped.PoseStamped'>": parse_posestamped,
                             "<class 'sensor_msgs.msg._PointCloud.PointCloud'>": parse_pointcloud2,
                             "<class 'sensor_msgs.msg._PointCloud2.PointCloud2'>": parse_pointcloud2}

        # Set up dict of transformer functions for certain message types/classes.
        self._msg_transformers = {"<class 'geometry_msgs.msg._Point.Point'>": do_transform_point,
                                  "<class 'geometry_msgs.msg._PointStamped.PointStamped'>": do_transform_point,
                                  "<class 'geometry_msgs.msg._Pose.Pose'>": do_transform_pose,
                                  "<class 'geometry_msgs.msg._PoseStamped.PoseStamped'>": do_transform_pose,
                                  "<class 'sensor_msgs.msg._PointCloud.PointCloud'>": do_transform_cloud,
                                  "<class 'sensor_msgs.msg._PointCloud2.PointCloud2'>": do_transform_cloud}

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

    def _transform_msg(self, msg, transform, msg_type=None):
        # First try using a known transformer for a specified msg_type.
        try:
            assert msg_type
            msg_class = str(roslib.message.get_message_class(msg_type))
            transformed_msg = self._msg_transformers[msg_class](msg, transform)
            return transformed_msg
        except:
            pass

        # Next, try to select a known transformer by checking the type of message.
        try:
            msg_class = str(type(msg))
            transformed_msg = self._msg_transformers[msg_class](msg, transform)
            return transformed_msg
        except:
            pass

        # Next, try each message type parser in succession and see if something sticks.
        for _, transformer in self._msg_transformers.items():
            try:
                transformed_msg = transformer(msg, transform)
                return transformed_msg
            except:
                pass

        # Finally, if none of the above stuck, raise an error.
        raise RuntimeError('Failed to find suitable transformer for given message type!')

    def execute(self, userdata):

        {{ render_execute_callbacks() }}

        # Parse msg
        try:
            if 'msg_type' in self._input_keys:
                parsed_msg = self._parse_msg(userdata.msg, msg_type=userdata.msg_type)
            else:
                parsed_msg = self._parse_msg(userdata.msg)
        except Exception as e:
            rospy.logerr('Failed to parse message: '.format(repr(e)))
            return 'aborted'

        # Parse transform
        if 'transform' in self._input_keys:
            try:
                transform = parse_transformstamped(userdata.transform)
            except:
                rospy.logerr('Failed to parse transform: ' + repr(e))
                return 'aborted'
        else:
            # Try getting the target frame from userdata
            if 'frame_id' in self._input_keys:
                target = userdata.frame_id
            elif 'target' in self._input_keys:
                target = userdata.target
            else:
                rospy.logerr("Failed to parse transform target ('target' or 'frame_id' should be specified in userdata input keys)!")
                return 'aborted'

            # Try getting the source frame from userdata or from the message header
            if 'source' in self._input_keys:
                source = userdata.source
            else:
                try:
                    source = parsed_msg.header.frame_id
                except:
                    rospy.logerr("Failed to parse transform source (should be specified as 'source' in userdata or 'frame_id' in message header)!")
                    return 'aborted'

            # Try looking up the transform using tf2
            try:
                while not (self._tf2_buffer.can_transform(target, source, rospy.Time(0))):
                    pass
                transform = self._tf2_buffer.lookup_transform(target, source, rospy.Time(0))
            except Exception as e:
                rospy.logerr("Failed to look up transform from source frame '{}' to target frame '{}': ".format(source, target, repr(e)))
                return 'aborted'

        # Adjust the transform based on the specified transform type
        if self._transform_type == 'translation':
            transform.transform.rotation.x = 0.0
            transform.transform.rotation.y = 0.0
            transform.transform.rotation.z = 0.0
            transform.transform.rotation.w = 1.0
        elif self._transform_type == 'rotation':
            transform.transform.translation.x = 0.0
            transform.transform.translation.y = 0.0
            transform.transform.translation.y = 0.0
        else:
            pass

        # Transform the msg
        try:
            if 'msg_type' in self._input_keys:
                transformed_msg = self._transform_msg(parsed_msg, transform, msg_type=userdata.msg_type)
            else:
                transformed_msg = self._transform_msg(parsed_msg, transform)
        except Exception as e:
            rospy.logerr('Error when transforming msg: ' + repr(e))
            return 'aborted'

        # Set point output key if specified
        for output_key in ['msg', 'output_msg', 'msg_output']:
            if output_key in self._output_keys:
                setattr(userdata, output_key, transformed_msg)

        # Set transform output key if specified
        for output_key in ['transform', 'output_transform', 'transform_output']:
            if output_key in self._output_keys:
                setattr(userdata, output_key, transform)

        return 'succeeded'
{% do defined_headers.append('class_TransformMsgState') %}{% endif %}
{% endblock class_defs %}

{% block body %}
smach.{{ parent_type }}.add('{{ name }}',
        {{ '' | indent(23, true) }}TransformMsgState({% if transform_type is defined %}transform_type = '{{ transform_type }}'{% endif %}{% if input_keys is defined %}{% if transform_type is defined %}, {% endif %}{{ render_input_keys(input_keys, indent=0) }}{% endif %}{% if output_keys is defined %}{% if transform_type is defined or input_keys is defined %}, {% endif %}{{ render_output_keys(output_keys, indent=0) }}{% endif %}{% if callbacks is defined %}, {{ render_callbacks(name, uuid, callbacks, indent=0) }}{% endif %}){% if transitions is defined %},
{{ render_transitions(transitions) }}{% endif %}{% if remapping is defined %},
{{ render_remapping(remapping) }}{% endif %})
{% endblock body %}
