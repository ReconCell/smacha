{% extends "Base.tpl" %}

{% from "Utils.tpl" import from_import %}

{% block imports %}
{{ super() }}
{{ from_import(defined_headers, 'smacha.srv', 'GripperSrv') }}
{{ from_import(defined_headers, 'smacha.srv', 'GripperSrvRequest') }}
{{ from_import(defined_headers, 'smacha.srv', 'GripperSrvResponse') }}
{{ from_import(defined_headers, 'geometry_msgs.msg', '*') }}
{% endblock imports %}

{% block defs %}
{{ super() }}
def gripper_srv(req):
    if req.max_effort > 5.0:
        print('gripper_srv() returning True')
        return GripperSrvResponse(True)
    else:
        print('gripper_srv() returning False')
        return GripperSrvResponse(False)
{% endblock defs %}

{% block main_def %}
{{ super() }}
    # Register a gripper service
    s = rospy.Service('gripper_srv', GripperSrv, gripper_srv)
{% endblock main_def %}

{% block base_footer %}
{{ super() }}
    rospy.signal_shutdown('All done.')
{% endblock base_footer %}
