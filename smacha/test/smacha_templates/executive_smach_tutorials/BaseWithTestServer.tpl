{% extends "Base.tpl" %}

{% from "Utils.tpl" import from_import %}

{% block imports %}
{{ super() }}
{{ from_import(defined_headers, 'smacha.msg', 'TestAction') }}
{{ from_import(defined_headers, 'actionlib_msgs.msg', '*') }}
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
