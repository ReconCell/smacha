{% from "Utils.tpl" import import_module, render_transitions, render_remapping, render_input_keys, render_output_keys, render_outcomes, render_def_lambda_callbacks, render_init_callbacks, render_execute_callbacks, render_callbacks %}

{% include "State.tpl" %}

{% block imports %}
{{ import_module(defined_headers, 'random') }}
{% endblock imports %}

{% block defs %}
{% if callbacks is defined %}
{% if input_keys is defined %}
{{ render_def_lambda_callbacks(defined_headers, name, uuid, input_keys, callbacks) }}
{% else %}
{{ render_def_lambda_callbacks(defined_headers, name, uuid, [], callbacks) }}
{% endif %}
{% endif %}
{% endblock defs %}

{% block class_defs %}
{% if 'class_RandomOutcomeState' not in defined_headers %}
class RandomOutcomeState(smach.State):
    def __init__(self, input_keys = [], output_keys = ['outcome'], callbacks = [], outcomes=['succeeded']):
        smach.State.__init__(self, input_keys=input_keys, output_keys=output_keys, outcomes=outcomes)

        {{ render_init_callbacks() }}
    
    def execute(self, userdata):
        
        {{ render_execute_callbacks() }}

        # Select random outcome
        random_outcome = random.choice(list(self._outcomes))

        # Set the outcome output key
        userdata.outcome = random_outcome
        
        return random_outcome
{% do defined_headers.append('class_RandomOutcomeState') %}{% endif %}
{% endblock class_defs %}

{% block header %}
{% endblock header %}

{% block body %}
smach.{{ parent_type }}.add('{{ name }}',
        {{ '' | indent(23, true) }}RandomOutcomeState({% if input_keys is defined %}{{ render_input_keys(input_keys, indent=0) }}{% endif %}{% if output_keys is defined %}{% if input_keys is defined %}, {% endif %}{{ render_output_keys(output_keys, indent=0) }}{% endif %}{% if callbacks is defined %}, {{ render_callbacks(name, uuid, callbacks, indent=0) }}{% endif %}{% if outcomes is defined %}{% if input_keys is defined or output_keys is defined or callbacks is defined %}, {% endif %}{{ render_outcomes(outcomes, indent=0) }}{% endif %}){% if transitions is defined %},
{{ render_transitions(transitions) }}{% endif %}{% if remapping is defined %},
{{ render_remapping(remapping) }}{% endif %})
{% endblock body %}
