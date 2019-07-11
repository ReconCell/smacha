{% from "Utils.tpl.py" import import_module, render_transitions, render_remapping, render_input_keys, render_output_keys, render_def_lambda_callbacks, render_init_callbacks, render_execute_callbacks, render_callbacks, render_userdata %}

{% block imports %}
{{ import_module(defined_headers, 'random') }}
{% endblock imports %}

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
{% endblock defs %}

{% block class_defs %}
{% if 'class_Foo' not in defined_headers %}
class Foo(smach.State):
    def __init__(self, name, input_keys=[], output_keys=[], callbacks=[]):
        smach.State.__init__(self, input_keys=input_keys, output_keys=output_keys, outcomes=['succeeded'])

        self._name = name

        {{ render_init_callbacks() }}

    def execute(self, userdata):
        for input_key in self._input_keys:
            smach.loginfo('Userdata input key \'{}\' BEFORE callback execution: {}'.format(input_key, userdata[input_key]))

        {{ render_execute_callbacks() }}

        for input_key in self._input_keys:
            smach.loginfo('Userdata input key \'{}\' AFTER callback execution: {}'.format(input_key, userdata[input_key]))

        return 'succeeded'
{% do defined_headers.append('class_Foo') %}{% endif %}
{% endblock class_defs %}

{% block cb_defs %}
{% if callbacks is defined %}
{% if input_keys is defined %}
{{ render_def_lambda_callbacks(defined_headers, 'Foo', name, uuid, input_keys, outcomes, callbacks) }}
{% else %}
{{ render_def_lambda_callbacks(defined_headers, 'Foo', name, uuid, [], outcomes, callbacks) }}
{% endif %}
{% endif %}
{% endblock cb_defs %}

{% block header_userdata %}
{% if userdata is defined %}{{ render_userdata(parent_sm_name, userdata) }}{% endif %}
{% endblock header_userdata %}

{% block body %}
smach.{{ parent_type }}.add('{{ name }}', Foo('{{ name }}'{% if input_keys is defined %}, {{ render_input_keys(input_keys, indent=0) }}{% endif %}{% if output_keys is defined %}, {{ render_output_keys(output_keys, indent=0) }}{% endif %}{% if callbacks is defined %}, {{ render_callbacks(name, uuid, callbacks, indent=0) }}{% endif %}){% if transitions is defined %}, 
{{ render_transitions(transitions) }}{% endif %}{% if remapping is defined %},
{{ render_remapping(remapping) }}{% endif %})
{% endblock body %}
