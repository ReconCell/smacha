{% block meta %}
name: TransformStamped
description:
  SMACH template that provides a TransformStamped helper function for, e.g.
  PublishMsgState. The function takes an input argument that can be specified
  as either a geometry_msgs.msg.Transform type, a
  geometry_msgs.msg.TransformStamped type or as a [[3],[4]] list, and returns a
  geometry_msgs.msg.TransformStamped type.
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
{{ from_import(defined_headers, 'geometry_msgs.msg', 'Transform') }}
{{ from_import(defined_headers, 'geometry_msgs.msg', 'TransformStamped') }}
{{ from_import(defined_headers, 'geometry_msgs.msg', 'Vector3') }}
{{ from_import(defined_headers, 'geometry_msgs.msg', 'Quaternion') }}
{% endblock imports %}

{% block defs %}
{% if 'def_parse_transformstamped' not in defined_headers %}
def parse_transformstamped(transform_input):
    """
    Parse transform_input into TransformStamped.
    """
    try:
        assert isinstance(transform_input, TransformStamped)
        return transform_input
    except:
        pass
    try:
        assert isinstance(transform_input, Transform)
        transform = TransformStamped(transform = transform_input)
        transform.header.stamp = rospy.Time.now()
        return transform
    except:
        pass
    try:
        transform = transform_input
        translation = Vector3(x=transform[0][0], y=transform[0][1], z=transform[0][2])
        rotation = Quaternion(x=transform[1][0], y=transform[1][1], z=transform[1][2], w=transform[1][3])
        transform = TransformStamped(transform = Transform(translation=translation, rotation=rotation))
        transform.header.stamp = rospy.Time.now()
        return transform
    except Exception as e:
        raise ValueError('Transform not properly specified (should be Transform, TransformStamped or [[3],[4]] list type)!')
{% do defined_headers.append('def_parse_transformstamped') %}{% endif %}
{% endblock defs %}
