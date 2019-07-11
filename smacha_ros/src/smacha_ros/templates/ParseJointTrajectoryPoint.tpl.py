{% block meta %}
name: ParseJointTrajectoryPoint
description:
  SMACH template that provides a ParseJointTrajectoryPoint helper function for,
  e.g. PublishMsgState. The function takes an input argument that can be
  specified as either a trajectory_msgs.msg.JointTrajectoryPoint type, a
  sensor_msgs.msg.JointState type or a [n] list type. and returns a
  trajectory_msgs.msg.JointTrajectoryPoint type.
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
{{ from_import(defined_headers, 'sensor_msgs.msg', 'JointState') }}
{{ from_import(defined_headers, 'trajectory_msgs.msg', 'JointTrajectoryPoint') }}
{% endblock imports %}

{% block defs %}
{% if 'def_parse_jointtrajectorypoint' not in defined_headers %}
def parse_jointtrajectorypoint(jointtrajectorypoint_input):
    """
    Parse jointtrajectorypoint_input into a JointTrajectoryPoint.
    """
    try:
        assert isinstance(jointtrajectorypoint_input, JointTrajectoryPoint)
        return jointtrajectorypoint_input
    except:
        pass
    try:
        assert isinstance(jointtrajectorypoint_input, JointState)
        positions = jointtrajectorypoint_input.position
        velocities = jointtrajectorypoint_input.velocity
        effort = jointtrajectorypoint_input.effort
        jointtrajpoint = JointTrajectoryPoint(positions=positions, velocities=velocities, effort=effort)
        return jointtrajpoint
    except:
        pass
    try:
        assert isinstance(jointtrajectorypoint_input, list)
        for position in jointtrajectorypoint_input:
            assert isinstance(position, (int,long,float))
        jointtrajpoint = JointTrajectoryPoint(positions=jointtrajectorypoint_input)
        return jointtrajpoint
    except Exception as e:
        rospy.logerr('Joint trajectory point not properly specified (should be JointTrajectoryPoint, JointState or [n] list type): {}'.format(repr(e)))
        raise
{% do defined_headers.append('def_parse_jointtrajectorypoint') %}{% endif %}
{% endblock defs %}
