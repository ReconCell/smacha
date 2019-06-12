{% block meta %}
name: ParsePointCloud
description:
  SMACH template that provides a ParsePointCloud helper function for, e.g.
  PublishMsgState. The function takes an input argument that can be specified
  as either a sensor_msgs.msg.PointCloud or sensor_msgs.msg.PointCloud2 type,
  and returns a sensor_msgs.msg.PointCloud type.
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

{% from "Utils.tpl.py" import from_import, from_import_as %}

{% block imports %}
{{ from_import(defined_headers, 'sensor_msgs.msg', 'PointCloud') }}
{{ from_import(defined_headers, 'geometry_msgs.msg', 'Point32') }}
{{ from_import_as(defined_headers, 'sensor_msgs', 'point_cloud2', 'pc2') }}
{% endblock imports %}

{% block defs %}
{% if 'def_parse_pointcloud' not in defined_headers %}
def parse_pointcloud(pointcloud_input):
    """
    Parse pointcloud_input into PointCloud.
    """
    try:
        assert isinstance(pointcloud_input, PointCloud)
        return pointcloud_input
    except:
        pass
    try:
        points = pc2.read_points(pointcloud_input, skip_nans=True, field_names=('x', 'y', 'z'))
        return PointCloud(points = map(lambda point: Point32(*point), points))
    except Exception as e:
        raise ValueError('Point cloud not properly specified (should be PointCloud or PointCloud2 type): ' + repr(e))
{% do defined_headers.append('def_parse_pointcloud') %}{% endif %}
{% endblock defs %}
