{% block meta %}
name: RandomOutcomeState
description: SMACH state randomly selects an outcome from a list of possible outcomes.
language: Python
framework: SMACH
type: State
tags: [core]
includes:
  - State
extends: []
variables: []
input_keys: []
output_keys:
- outcome:
    description: The selected outcome is stored in the outcome output_key.
    type: str
outcomes:
- User-specified (default = succeeded)
{% endblock meta %}

{% from "Utils.tpl.py" import import_module, render_init_callbacks, render_execute_callbacks %}

{% extends "State.tpl.py" %}

{% block imports %}
{{ super() }}
{{ import_module(defined_headers, 'random') }}
{% endblock imports %}

{% block class_defs %}
{{ super() }}
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
