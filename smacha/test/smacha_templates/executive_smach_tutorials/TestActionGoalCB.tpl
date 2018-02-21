{% extends "SimpleActionState.tpl" %}

{% from "Utils.tpl" import from_import %}

{% block imports %}
{{ from_import(defined_headers, 'smacha.msg', 'TestAction') }}
{{ from_import(defined_headers, 'smacha.msg', 'TestGoal') }}
{{ super() }}
{% endblock imports %}

{% block header %}
{% endblock header %}

{% block body %}
# Add another simple action state. This will give a goal
# that should abort the action state when it is received, so we
# map 'aborted' for this state onto 'succeeded' for the state machine.
def goal_callback(userdata, default_goal):
    goal = TestGoal()
    goal.goal = 2
    return goal
{{ super() }}
{% endblock body %}
