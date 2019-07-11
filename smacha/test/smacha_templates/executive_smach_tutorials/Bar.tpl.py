{% from "Utils.tpl.py" import render_transitions, render_remapping %}

{% block class_defs %}
# define state Bar
class Bar(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['outcome1']{% if remapping is defined %},
                             input_keys=['bar_counter_in']{% endif %})

    def execute(self, userdata):
        smach.loginfo('Executing state {{ name }}'){% if remapping is defined %}
        smach.loginfo('Counter = %f'%userdata.bar_counter_in){% endif %} 
        return 'outcome1'
{% endblock class_defs %}

{% block body %}
smach.{{ parent_type }}.add('{{ name }}', Bar(){% if transitions is defined %}, 
{{ render_transitions(transitions) }}{% endif %}{% if remapping is defined %},
{{ render_remapping(remapping) }}{% endif %})
{% endblock body %}
