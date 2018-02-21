{% extends "ServiceState.tpl" %}

{% from "Utils.tpl" import from_import %}

{% block imports %}
{{ from_import(defined_headers, 'smacha.srv', 'GripperSrvRequest') }}
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
