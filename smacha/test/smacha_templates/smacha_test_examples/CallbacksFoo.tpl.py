{% from "Utils.tpl" import render_transitions, render_remapping, render_input_keys, render_output_keys, render_init_callbacks, render_execute_callbacks, render_callbacks %}

{% block defs %}
{% if 'foo_animals_cb' not in defined_headers %}
# Define normal callback for 'animals' output key
@smach.cb_interface(input_keys=['animals'],
                    output_keys=['animals'],
                    outcomes=['succeeded'])
def foo_animals_cb(userdata):
    userdata['animals'].append('turtles')
    return 'succeeded'
{% do defined_headers.append('foo_animals_cb') %}{% endif %}

{% if 'foo_numbers_cb' not in defined_headers %}
# Define normal callback for 'numbers' output key
@smach.cb_interface(input_keys=['numbers'],
                    output_keys=['numbers'],
                    outcomes=['succeeded'])
def foo_numbers_cb(userdata):
    userdata['numbers'].append(userdata['numbers'][-1]+1)
    return 'succeeded'
{% do defined_headers.append('foo_numbers_cb') %}{% endif %}

{% if callbacks is defined %}
{% for cb_output_key, cb in callbacks.iteritems() %}
{% if cb is expression %}
{% set cb_name = name|lower + '_' + cb_output_key|lower + '_lambda_cb' %}
{% if cb_name not in defined_headers %}
@smach.cb_interface(input_keys={{ input_keys }}, 
                    output_keys=['{{ cb_output_key }}'],
                    outcomes=['succeeded'])
def {{ cb_name }}(userdata):
    lambda_cb = {{ cb | exptostr }}
    userdata['{{ cb_output_key }}'] = lambda_cb(userdata)
    return 'succeeded'
{% do defined_headers.append(cb_name) %}{% endif %}
{% endif %}
{% endfor %}
{% endif %}
{% endblock defs %}

{% block class_defs %}
{% if 'class_foo' not in defined_headers %}
class Foo(smach.State):
    def __init__(self, name, input_keys=None, output_keys=None, callbacks=None):
        smach.State.__init__(self, input_keys=input_keys, output_keys=output_keys, outcomes=['succeeded'])

        self._name = name
        
        {{ render_init_callbacks() }}
        
    def execute(self, userdata):
        for input_key in self._input_keys:
            rospy.loginfo('userdata[\'{}\'] BEFORE callback execution: {}'.format(input_key, userdata[input_key]))

        {{ render_execute_callbacks() }}

        for output_key in self._output_keys:
            rospy.loginfo('userdata[\'{}\'] AFTER callback execution: {}'.format(output_key, userdata[output_key]))

        return 'succeeded'
{% do defined_headers.append('class_foo') %}{% endif %}
{% endblock class_defs %}

{% block body %}
smach.{{ parent_type }}.add('{{ name }}', Foo('{{ name }}'{% if input_keys is defined %}, {{ render_input_keys(input_keys, indent=0) }}{% endif %}{% if output_keys is defined %}, {{ render_output_keys(output_keys, indent=0) }}{% endif %}{% if callbacks is defined %}, {{ render_callbacks(name, callbacks, indent=0) }}{% endif %}){% if transitions is defined %}, 
{{ render_transitions(transitions) }}{% endif %}{% if remapping is defined %},
{{ render_remapping(remapping) }}{% endif %})
{% endblock body %}
