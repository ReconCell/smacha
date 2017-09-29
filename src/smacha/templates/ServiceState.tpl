{% from "Utils.tpl" import render_request_slots, render_input_keys, render_response_slots, render_output_keys, render_transitions, render_remapping %}

{% include "State.tpl" %}

{% block header %}
{% endblock header %}

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
