{% from "Utils.tpl" import render_userdata %}
{% set defined_headers = [] %}
{% set local_vars = [] %}
{% block base_header %}
#!/usr/bin/env python
{{ base_header }}
{% endblock base_header %}

{% block imports %}
import roslib; {% if manifest is defined %}roslib.load_manifest('{{ manifest }}'){% endif %}
import rospy
import smach
import smach_ros
{{ imports }}
{% endblock imports %}

{% block defs %}
{{ defs }}
{% endblock defs %}

{% block class_defs %}
{{ class_defs }}
{% endblock class_defs %}

{% if name is defined %}{% set sm_name = name | lower() %}{% else %}{% set sm_name = 'sm' %}{% endif %}

{% block main_def %}
def {% if function_name is defined %}{{ function_name | lower() }}{% else %}main{% endif %}():
    rospy.init_node('{% if node_name is defined %}{{ node_name }}{% else %}{{ name }}{% endif %}')

    {{ main_def | indent(4) }}
{% endblock main_def %}
   
{% block body %}
    {{ sm_name }} = smach.StateMachine(outcomes=[{% for outcome in outcomes %}'{{ outcome }}'{% if not loop.last %}, {% endif %}{% endfor %}])

    {% if userdata is defined %}{{ render_userdata(name | lower(), userdata) | indent(4) }}{% endif %}
    {% if name in header %}{{ header[name] | indent(4, true) }}{% endif %}

    with {{ sm_name }}:

        {{ body | indent(8) }}
{% endblock body %}

{% block footer %}
        {{ footer | indent(8) }}
{% endblock footer %}

{% block introspection_server %}
    sis = smach_ros.IntrospectionServer('{% if node_name is defined %}{{ node_name }}{% else %}{{ name }}{% endif %}', {{ name | lower() }}, '/{{ name }}')
    sis.start()
{% endblock introspection_server %}

{% block execute %}
    {{ execute | indent(4) }}

    outcome = {{ sm_name }}.execute()
{% endblock execute %}    

{% block spin %}   
    rospy.spin()
{% endblock spin %}

{% block base_footer %}
    {{ base_footer | indent(4) }}
{% endblock base_footer %}

{% block main %}
if __name__ == '__main__':
{{ '' | indent(4, true) }}{% if function_name is defined %}{{ function_name | lower() }}{% else %}main{% endif %}()
{% endblock main %}
