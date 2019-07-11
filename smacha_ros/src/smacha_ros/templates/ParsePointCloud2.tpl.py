{% block meta %}
name: ParsePointCloud2
description:
  SMACH template that provides a ParsePointCloud helper function for, e.g.
  PublishMsgState. The function takes an input argument that can be specified
  as either a sensor_msgs.msg.PointCloud or sensor_msgs.msg.PointCloud2 type,
  and returns a sensor_msgs.msg.PointCloud2 type.
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
{{ from_import(defined_headers, 'sensor_msgs.msg', 'PointCloud') }}
{{ from_import(defined_headers, 'sensor_msgs.msg', 'PointCloud2') }}
{{ from_import(defined_headers, 'sensor_msgs.point_cloud2', 'create_cloud_xyz32') }}
{% endblock imports %}

{% block defs %}
{% if 'def_parse_pointcloud2' not in defined_headers %}
def parse_pointcloud2(pointcloud_input):
    """
    Parse pointcloud_input into PointCloud2.
    """
    try:
        assert isinstance(pointcloud_input, PointCloud2)
        return pointcloud_input
    except:
        pass
    try:
        points = [[point.x, point.y, point.z] for point in pointcloud_input.points]
        pointcloud2 = create_cloud_xyz32(header=pointcloud_input.header, points=points)
        return pointcloud2
    except:
        raise ValueError('Point cloud not properly specified (should be PointCloud or PointCloud2 type)!')
{% do defined_headers.append('def_parse_pointcloud2') %}{% endif %}
{% endblock defs %}
