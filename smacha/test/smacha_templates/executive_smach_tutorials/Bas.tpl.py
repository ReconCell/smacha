{% from "Utils.tpl.py" import render_transitions %}

{% block class_defs %}
# define state Bas
class Bas(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome3'])

    def execute(self, userdata):
        print('Executing state {{ name }}')
        return 'outcome3'
{% endblock class_defs %}

{% block body %}
smach.{{ parent_type }}.add('{{ name }}', Bas(),
{{ render_transitions(transitions) }})
{% endblock body %}
