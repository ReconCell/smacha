{% block meta %}
name: ServiceState
description: SMACH ServiceState for calling services.
language: Python
framework: SMACH
type: Container
tags: [core]
includes:
  - State
extends: []
variables:
- service_namespace:
    description: The ROS topic/namespace of the service, e.g. '/robot_1/gripper_service'.
    type: str
- service:
    description: The service type.
    type: str
- - request:
      description: The request as defined by a .srv file.
      type: str
  - request_slots:
      description: Request slots as defined by userdata.
      type: list
  - request_cb:
      description: A request callback as defined either by a lambda function in the script or in an inheriting template.
      type: [function, str]
- - response_slots:
      description: Response slots as defined by userdata.
      type: list
  - response_cb:
      description: A response callback as defined either by a lambda function in the script or in an inheriting template.
      type: [function, str]
- - input_keys:
      description: The names of the userdata input keys needed for the request callback.
      type: list
- - output_keys:
      description: The names of the userdata output keys needed for the response callback.
      type: list
- - userdata:
      description: The definitions for the userdata keys named in the input_keys and output_keys variables.
      type: dict
{% endblock meta %}

{% from "Utils.tpl.py" import render_request_slots, render_input_keys, render_response_slots, render_output_keys, render_transitions, render_remapping %}

{% include "State.tpl.py" %}

{% block body %}
smach.{{ parent_type }}.add('{{ name }}',
{{ '' | indent(23, true) }}smach_ros.ServiceState('{{ service_namespace }}', {{ service }}{% if request is defined %},
{{ 'request = ' | indent(51, true) }}{{ request }}{% elif request_slots is defined %},
{{ render_request_slots(request_slots) }}{% elif request_cb is defined %},
{{ 'request_cb = ' | indent(51, true) }}{{ request_cb }}{% if input_keys is defined %},
{{ render_input_keys(input_keys) }}{% endif %}{% endif %}{% if response_slots is defined %},
{{ render_response_slots(response_slots) }}{% elif response_cb is defined %},
{{ 'response_cb = ' | indent(51, true) }}{{ response_cb }}{% if output_keys is defined %},
{{ render_output_keys(output_keys) }}{% endif %}{% endif %}){% if transitions is defined %},
{{ render_transitions(transitions) }}{% endif %}{% if remapping is defined %},
{{ render_remapping(remapping) }}{% endif %})
{% endblock body %}
