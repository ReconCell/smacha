{% block meta %}
name: RandomOutcomeState
description:
  SMACH state randomly selects an outcome from a list of possible outcomes.
language: Python
framework: SMACH
type: State
tags: [core]
includes: []
extends:
- State
variables:
- - input_keys:
      description:
        The names of the userdata input keys to be used by the optionally
        specified callback functions.
      type: list of str
- - output_keys:
      description:
        The names of the userdata output keys corresponding to each
        optionally specified callback function.
      type: list of str
- - callbacks:
      description: >
        Either callback function names or backtick-wrapped lambda functions
        for possible modifications to the randomization procedure.
        The default callback is
        outcome: '`lambda ud.outcome: random.choice(list(self._outcomes))`'.
      type: dict of str
- outcomes:
    description: The possible outcomes.
    type: list of str
input_keys:
- outcome:
    description: The selected outcome is stored in the outcome output_key.
    type: str
output_keys:
- outcome:
    description: The selected outcome is stored in the outcome output_key.
    type: str
outcomes:
- succeeded
{% endblock meta %}

{% from "Utils.tpl.py" import import_module, render_init_callbacks, render_execute_callbacks %}

{% if callbacks is not defined %}
{% set callbacks = {'outcome': '`lambda ud: random.choice(list(self._outcomes))`'} %}
{% endif %}

{% extends "State.tpl.py" %}

{% block imports %}
{{ super() }}
{{ import_module(defined_headers, 'random') }}
{% endblock imports %}

{% block class_defs %}
{{ super() }}
{% if 'class_RandomOutcomeState' not in defined_headers %}
class RandomOutcomeState(smach.State):
    def __init__(self, input_keys = ['outcome'], output_keys = ['outcome'], callbacks = {}, outcomes=['succeeded']):
        smach.State.__init__(self, input_keys=input_keys, output_keys=output_keys, outcomes=outcomes)

        {{ render_init_callbacks() }}

    def execute(self, userdata):

        {{ render_execute_callbacks() }}

        return userdata.outcome
{% do defined_headers.append('class_RandomOutcomeState') %}{% endif %}
{% endblock class_defs %}
