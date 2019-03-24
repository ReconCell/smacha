{% block meta %}
name: ParsePointStamped
description:
  SMACH template that provides a ParsePointStamped helper function for, e.g.
  PublishMsgState. The function takes an input argument that can be specified
  as either a geometry_msgs.msg.Point type, a geometry_msgs.msg.PointStamped
  type or as a [3] list, and returns a geometry_msgs.msg.Point type.
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
{{ from_import(defined_headers, 'geometry_msgs.msg', 'Point') }}
{{ from_import(defined_headers, 'geometry_msgs.msg', 'Point32') }}
{{ from_import(defined_headers, 'geometry_msgs.msg', 'PointStamped') }}
{% endblock imports %}

{% block defs %}
{% if 'def_parse_pointstamped' not in defined_headers %}
def parse_pointstamped(point_input):
    """
    Parse point_input into PointStamped.
    """
    try:
        assert isinstance(point_input, PointStamped)
        return point_input
    except:
        pass
    try:
        assert isinstance(point_input, Point)
        point = PointStamped(point = point_input)
        point.header.stamp = rospy.Time.now()
        return point
    except:
        pass
    try:
        assert isinstance(point_input, Point32)
        point = PointStamped(point = Point(x=point_input.x, y=point_input.y, z=point_input.z))
        point.header.stamp = rospy.Time.now()
        return point
    except:
        pass
    try:
        point = point_input
        point = PointStamped(point = Point(x=point[0], y=point[1], z=point[2]))
        point.header.stamp = rospy.Time.now()
        return point
    except Exception as e:
        raise ValueError('Point not properly specified (should be Point, PointStamped or [3] list type)!')
{% do defined_headers.append('def_parse_pointstamped') %}{% endif %}
{% endblock defs %}
