{% block meta %}
name: ParsePoseArray
description:
  SMACH template that provides a ParsePoseArray helper function for, e.g.
  PublishMsgState. The function takes an input argument that can be specified
  as a geometry_msgs.msg.PoseArray type or as a list of geometry_msgs.msg.Pose
  types, geometry_msgs.msg.PoseStamped types or [[3],[4]] lists, and returns a
  geometry_msgs.msg.PoseArray type.
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
{{ from_import(defined_headers, 'geometry_msgs.msg', 'PoseArray') }}
{{ from_import(defined_headers, 'geometry_msgs.msg', 'Point') }}
{{ from_import(defined_headers, 'geometry_msgs.msg', 'Quaternion') }}
{% endblock imports %}

{% block defs %}
{% if 'def_parse_posearray' not in defined_headers %}
def parse_posearray(posearray_input):
    """
    Parse posearray_input into a PoseArray.
    """
    try:
        assert isinstance(posearray_input, PoseArray)
        return posearray_input
    except:
        pass
    try:
        assert isinstance(posearray_input, list)
        posearray = PoseArray()
        for pose in posearray_input:
            try:
                assert isinstance(pose, Pose)
                posearray.poses.append(pose)
                continue
            except:
                pass
            try:
                assert isinstance(pose, PoseStamped)
                posearray.poses.append(pose.pose)
                continue
            except:
                pass
            try:
                position = Point(x=pose[0][0], y=pose[0][1], z=pose[0][2])
                orientation = Quaternion(x=pose[1][0], y=pose[1][1], z=pose[1][2], w=pose[1][3])
                pose = Pose(position=position, orientation=orientation)
                posearray.poses.append(pose)
                continue
            except Exception as e:
                raise ValueError('Pose in pose array input not properly specified (should be Pose, PoseStamped or [[3],[4]] list)!')
        posearray.header.stamp = rospy.Time.now()
        return posearray
    except Exception as e:
        raise ValueError('Pose array not properly specified (should be PoseArray or list of Pose, PoseStamped or [[3],[4]] list types)!')
{% do defined_headers.append('def_parse_posearray') %}{% endif %}
{% endblock defs %}
