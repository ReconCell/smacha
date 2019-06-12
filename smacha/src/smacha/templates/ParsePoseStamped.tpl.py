{% block meta %}
name: ParsePoseStamped
description:
  SMACH template that provides a ParsePoseStamped helper function for, e.g.
  PublishMsgState. The function takes an input argument that can be specified
  as either a geometry_msgs.msg.Pose type, a geometry_msgs.msg.PoseStamped type,
  or a [[3],[4]] list, and returns a geometry_msgs.msg.PoseStamped type.
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

{% from "Utils.tpl.py" import from_import %}

{% block imports %}
{{ from_import(defined_headers, 'geometry_msgs.msg', 'Pose') }}
{{ from_import(defined_headers, 'geometry_msgs.msg', 'PoseStamped') }}
{{ from_import(defined_headers, 'geometry_msgs.msg', 'Point') }}
{{ from_import(defined_headers, 'geometry_msgs.msg', 'Quaternion') }}
{% endblock imports %}

{% block defs %}
{% if 'def_parse_posestamped' not in defined_headers %}
def parse_posestamped(pose_input):
    """
    Parse pose_input into PoseStamped.
    """
    try:
        assert isinstance(pose_input, PoseStamped)
        return pose_input
    except:
        pass
    try:
        assert isinstance(pose_input, Pose)
        pose = PoseStamped(pose = pose_input)
        pose.header.stamp = rospy.Time.now()
        return pose
    except:
        pass
    try:
        pose = pose_input
        position = Point(x=pose_input[0][0], y=pose_input[0][1], z=pose_input[0][2])
        orientation = Quaternion(x=pose_input[1][0], y=pose_input[1][1], z=pose_input[1][2], w=pose_input[1][3])
        pose = PoseStamped(pose = Pose(position=position, orientation=orientation))
        pose.header.stamp = rospy.Time.now()
        return pose
    except Exception as e:
        raise ValueError('Pose not properly specified (should be Pose, PoseStamped or [[3],[4]] list)!')
{% do defined_headers.append('def_parse_posestamped') %}{% endif %}
{% endblock defs %}
