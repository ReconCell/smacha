{% from "Utils.tpl" import render_transitions, render_remapping, render_input_keys, render_output_keys, render_def_lambda_callbacks, render_init_callbacks, render_execute_callbacks, render_callbacks %}

{% include "State.tpl" %}

{% block imports %}
{% endblock imports %}

{% block defs %}
{% if callbacks is defined %}
{{ render_def_lambda_callbacks(defined_headers, name, input_keys, callbacks) }}
{% endif %}
{% endblock defs %}

{% block class_defs %}
{% if 'class_CallbacksState' not in defined_headers %}
class CallbacksState(smach.State):
    def __init__(self, input_keys = [], output_keys = [], callbacks = []):
        smach.State.__init__(self, input_keys=input_keys, output_keys=output_keys, outcomes=['succeeded'])

        {{ render_init_callbacks() }}
    
    def execute(self, userdata):
        
        {{ render_execute_callbacks() }}
        
        return 'succeeded'
{% do defined_headers.append('class_CallbacksState') %}{% endif %}
{% endblock class_defs %}

{% block header %}
{% endblock header %}

{% block body %}
smach.{{ parent_type }}.add('{{ name }}',
        {{ '' | indent(23, true) }}CallbacksState({% if input_keys is defined %}{{ render_input_keys(input_keys, indent=0) }}{% endif %}{% if output_keys is defined %}{% if input_keys is defined %}, {% endif %}{{ render_output_keys(output_keys, indent=0) }}{% endif %}{% if callbacks is defined %}, {{ render_callbacks(name, callbacks, indent=0) }}{% endif %}){% if transitions is defined %},
{{ render_transitions(transitions) }}{% endif %}{% if remapping is defined %},
{{ render_remapping(remapping) }}{% endif %})
{% endblock body %}