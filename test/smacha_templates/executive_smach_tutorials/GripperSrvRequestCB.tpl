{% extends "ServiceState.tpl" %}

{% block imports %}
{% if 'smacha_srv_import_GripperSrvRequest' not in defined_headers %}
from smacha.srv import GripperSrvRequest
{% do defined_headers.append('smacha_srv_import_GripperSrvRequest') %}
{% endif %}
{{ super() }}
{% endblock imports %}

{% block header %}
{% endblock header %}

{% block body %}
@smach.cb_interface(input_keys=['gripper_input'])
def gripper_request_cb(userdata, request):
   gripper_request = GripperSrvRequest()
   gripper_request.position.x = 2.0
   gripper_request.max_effort = userdata.gripper_input
   return gripper_request
{{ super() }}
{% endblock body %}