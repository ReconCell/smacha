{% extends "Base.tpl" %}

{% block imports %}
{{ super() }}
{% if 'smacha_srv_import_GripperSrv' not in defined_headers %}
from smacha.srv import GripperSrv
{% do defined_headers.append('smacha_srv_import_GripperSrv') %}
{% endif %}
{% if 'smacha_srv_import_GripperSrvRequest' not in defined_headers %}
from smacha.srv import GripperSrvRequest
{% do defined_headers.append('smacha_srv_import_GripperSrvRequest') %}
{% endif %}
{% if 'smacha_srv_import_GripperSrvResponse' not in defined_headers %}
from smacha.srv import GripperSrvResponse
{% do defined_headers.append('smacha_srv_import_GripperSrvResponse') %}
{% endif %}
{% if 'geometry_msgs_msg_import' not in defined_headers %}
from geometry_msgs.msg import *
{% do defined_headers.append('geometry_msgs_msg_import') %}
{% endif %}
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
