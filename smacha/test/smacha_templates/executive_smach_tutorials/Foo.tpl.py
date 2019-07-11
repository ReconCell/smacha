{% from "Utils.tpl.py" import render_transitions, render_remapping %}

{% block class_defs %}
# define state Foo
class Foo(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['outcome1','outcome2']{% if remapping is defined %},
                             input_keys=['foo_counter_in'],
                             output_keys=['foo_counter_out']){% else %})
        self.counter = 0{% endif %}

    def execute(self, userdata):
        smach.loginfo('Executing state {{ name }}'){% if remapping is defined %}
        if userdata.foo_counter_in < 3:
            userdata.foo_counter_out = userdata.foo_counter_in + 1{% else %}
        if self.counter < 3:
            self.counter += 1{% endif %}
            return 'outcome1'
        else:
            return 'outcome2'
{% endblock class_defs %}

{% block body %}
smach.{{ parent_type }}.add('{{ name }}', Foo(){% if transitions is defined %}, 
{{ render_transitions(transitions) }}{% endif %}{% if remapping is defined %},
{{ render_remapping(remapping) }}{% endif %})
{% endblock body %}
