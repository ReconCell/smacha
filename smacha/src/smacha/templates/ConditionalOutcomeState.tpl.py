{% block meta %}
name: ConditionalOutcomeState
description:
    A SMACH state that conditionally selects an outcome from a list of possible
    outcomes if its corresponding callback returns True.
language: Python
framework: SMACH
type: State
tags: [core]
includes: []
extends:
- State
variables:
- - outcomes:
      description: The possible outcomes (other than default).
      type: list of str
- - callbacks:
      description:
        Either callback function names or backtick-wrapped lambda function
        strings defining conditions that return True for corresponding outcomes.
      type: dict of str
- - input_keys:
      description:
        The names of the userdata input keys needed for the outcome conditional
        callbacks.
      type: list of str
input_keys: []
output_keys:
- outcome:
    description: The selected outcome is stored in the outcome output_key.
    type: str
outcomes:
- default:
    description:
        The default outcome, selected if no other outcome is selected, e.g. if
        all callbacks return False.
    type: str
{% endblock meta %}

{% from "Utils.tpl.py" import render_init_callbacks, render_execute_callbacks %}

{% extends "State.tpl.py" %}

{% block class_defs %}
{{ super() }}
{% if 'class_ConditionalOutcomeState' not in defined_headers %}
class ConditionalOutcomeState(smach.State):
    def __init__(self, input_keys = [], callbacks = [], outcomes=[]):

        # SMACH states store outcomes as a set, which is unordered,
        # so set up an ordered outcomes list.
        self._ordered_outcomes = outcomes

        # Ensure that there is a default outcome and that it is the last item
        # in the outcomes list.
        if 'default' in self._ordered_outcomes:
            self._ordered_outcomes.remove('default')
        self._ordered_outcomes.append('default')

        # Match input_keys and output_keys to outcomes.
        # NOTE: Because of this requirement, this state does not allow the user
        # to manually define output keys.
        output_keys = []
        for outcome in self._ordered_outcomes:
            if outcome != 'default':
                if outcome not in input_keys:
                    input_keys.append(outcome)
                output_keys.append(outcome)

        # Create a special output key named 'outcome' that returns the name
        # of the selected outcome
        output_keys.append('outcome')

        smach.State.__init__(self, input_keys=input_keys, output_keys=output_keys, outcomes=self._ordered_outcomes)

        {{ render_init_callbacks() }}

    def execute(self, userdata):

        {{ render_execute_callbacks() }}

        # Iterate through outcomes and return the first one with an output key
        # that returns True in the userdata.
        for outcome in self._ordered_outcomes:
            if outcome in self._output_keys and getattr(userdata, outcome):
                userdata.outcome = outcome
                return outcome

        return 'default'
{% do defined_headers.append('class_ConditionalOutcomeState') %}{% endif %}
{% endblock class_defs %}
