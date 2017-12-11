{% extends "Base.tpl" %}

{% block imports %}
{{ super() }}
{% if 'smacha_msg_import_TestAction' not in defined_headers %}
from smacha.msg import TestAction
{% do defined_headers.append('smacha_msg_import_TestAction') %}
{% endif %}
{% if 'actionlib_msgs_msg_import' not in defined_headers %}
from actionlib_msgs.msg import *
{% do defined_headers.append('actionlib_msgs_msg_import') %}
{% endif %}
{% endblock imports %}

{% block class_defs %}
{{ super() }}
# Create a trivial action server
class TestServer:
    def __init__(self,name):
        self._sas = SimpleActionServer(name,
                TestAction,
                execute_cb=self.execute_cb)

    def execute_cb(self, msg):
        if msg.goal == 0:
            self._sas.set_succeeded()
        elif msg.goal == 1:
            self._sas.set_aborted()
        elif msg.goal == 2:
            self._sas.set_preempted()
{% endblock class_defs %}

{% block main_def %}
{{ super() }}
    # Start an action server
    server = TestServer('test_action')
{% endblock main_def %}

{% block base_footer %}
{{ super() }}
    rospy.signal_shutdown('All done.')
{% endblock base_footer %}