{% extends "ServiceState.tpl.py" %}

{% from "Utils.tpl.py" import from_import %}

{% block imports %}
{{ from_import(defined_headers, 'smacha_ros.srv', 'GripperSrvRequest') }}
{{ super() }}
{% endblock imports %}

{% block body %}
@smach.cb_interface(input_keys=['gripper_input'])
def gripper_request_cb(userdata, request):
   gripper_request = GripperSrvRequest()
   gripper_request.position.x = 2.0
   gripper_request.max_effort = userdata.gripper_input
   return gripper_request
{{ super() }}
{% endblock body %}
