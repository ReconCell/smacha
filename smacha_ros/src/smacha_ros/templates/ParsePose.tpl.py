{% block meta %}
name: ParsePose
description:
  SMACH template that provides a ParsePose helper function for, e.g.
  PublishMsgState. The function takes an input argument that can be specified
  as either a geometry_msgs.msg.Pose type, a geometry_msgs.msg.PoseStamped
  type, or as a [[3],[4]] list, and returns a geometry_msgs.msg.Pose type.
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
{% if 'def_parse_pose' not in defined_headers %}
def parse_pose(pose_input):
    """
    Parse pose_input into Pose.
    """
    try:
        assert isinstance(pose_input, Pose)
        return pose_input
    except:
        pass
    try:
        assert isinstance(pose_input, PoseStamped)
        pose = pose_input.pose
        return pose
    except:
        pass
    try:
        pose = pose_input
        position = Point(x=pose_input[0][0], y=pose_input[0][1], z=pose_input[0][2])
        orientation = Quaternion(x=pose_input[1][0], y=pose_input[1][1], z=pose_input[1][2], w=pose_input[1][3])
        pose = Pose(position=position, orientation=orientation)
        return pose
    except Exception as e:
        raise ValueError('Pose not properly specified (should be Pose, PoseStamped or [[3],[4]] list)!')
{% do defined_headers.append('def_parse_pose') %}{% endif %}
{% endblock defs %}
