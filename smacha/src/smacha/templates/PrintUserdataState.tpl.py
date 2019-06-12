{% block meta %}
name: PrintUserdataState
description: SMACH state that prints the values of userdata input_keys to standard output.
language: Python
framework: SMACH
type: State
tags: [core]
includes: []
extends:
  - State
variables:
- input_keys:
    description:
      The names of the userdata input keys to be printed.
    type: list of str
- - output_keys:
      description:
        The names of the userdata output keys corresponding to each
        optionally specified callback function.
      type: list of str
- - callbacks:
      description:
        Either callback function names or backtick-wrapped lambda functions
        for possible modifications to the printing procedure.
      type: dict of str
- - outcomes:
      description: The possible outcomes.
      type: list of str
input_keys: []
output_keys: []
outcomes:
- succeeded
{% endblock meta %}

{% from "Utils.tpl.py" import import_module %}

{% extends "State.tpl.py" %}

{% block imports %}
{{ super() }}
{{ import_module(defined_headers, 'rospy') }}
{% endblock imports %}

{% block class_defs %}
{{ super() }}
{% if 'class_PrintUserdataState' not in defined_headers %}
class PrintUserdataState(smach.State):
    def __init__(self, input_keys = [], output_keys = [], callbacks = [], outcomes=['succeeded']):
        smach.State.__init__(self, input_keys=input_keys, output_keys=output_keys, outcomes=outcomes)

    def execute(self, userdata):

        # Print input keys to terminal
        for input_key in self._input_keys:
            rospy.loginfo('userdata.{}: {}'.format(input_key, userdata[input_key]))

        return 'succeeded'
{% do defined_headers.append('class_PrintUserdataState') %}{% endif %}
{% endblock class_defs %}
