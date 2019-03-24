{% block meta %}
name: WaitForMsgState
description: >
  SMACH template that provides a WaitForMsgState helper class for
  ReadTopicState.

  The class acts as a generic message listener with blocking, timeout, latch
  and flexible usage.

  It is meant to be extended with a case specific class that initializes this
  one appropriately and contains the msg_cb (or overrides execute if really
  needed).

  Its waitForMsg method implements the core functionality: waiting for the
  message, returning the message itself or None on timeout.

  Its execute method wraps the waitForMsg and returns succeeded or aborted,
  depending on the returned message being existent or None. Additionally, in
  the successful case, the msg_cb, if given, will be called with the message
  and the userdata, so that a self defined method can convert message data to
  smach userdata. Those userdata fields have to be passed via 'output_keys'.

  For more info, see:
      http://wiki.ros.org/executive_smach/AdditionalStateTypes
      https://github.com/felix-kolbe/uashh-rvl-ros-pkg/blob/master/uashh_smach/src/uashh_smach/util.py
language: Python
framework: SMACH
type: None
tags: [core]
includes: []
extends:
- State
variables:
- topic:
    description: The name of the topic from which the data should be read.
    type: str
- msg_type:
    description: The ROS message type of the topic.
    type: str
- - msg_cb:
      description: >
        If the state outcome should depend on the message content, the msg_cb
        can dictate the outcome:
        If the state outcome should depend on the message content, the msg_cb can dictate the outcome:
        If msg_cb returns True, execute() will return "succeeded".
        If msg_cb returns False, execute() will return "aborted".
        If msg_cb has no return statement, execute() will act as described above.
      type: str
- - latch:
      description:
        If True waitForMsg will return the last received message, so one
        message might be returned indefinite times.
      type: bool
- - timeout:
      description:
        Seconds to wait for a message, defaults to None, disabling timeout.
      type: int
- - output_keys:
      description:
        Userdata keys that the message callback needs to write to.
      type: list of str
input_keys: []
output_keys: []
outcomes:
- succeeded
- aborted
{% endblock meta %}

#
# Based on the original code described here:
# http://wiki.ros.org/executive_smach/AdditionalStateTypes
# https://github.com/felix-kolbe/uashh-rvl-ros-pkg/blob/master/uashh_smach/src/uashh_smach/util.py
#
{% from "Utils.tpl.py" import import_module, render_transitions, render_remapping, render_output_keys %}

{% extends "State.tpl.py" %}

{% block imports %}
{{ super() }}
{{ import_module(defined_headers, 'threading') }}
{% endblock imports %}

{% block class_defs %}
{{ super() }}
{% if 'class_WaitForMsgState' not in defined_headers %}
class WaitForMsgState(smach.State):
    """This class acts as a generic message listener with blocking, timeout, latch and flexible usage.

    It is meant to be extended with a case specific class that initializes this one appropriately
    and contains the msg_cb (or overrides execute if really needed).

    Its waitForMsg method implements the core functionality: waiting for the message, returning
    the message itself or None on timeout.

    Its execute method wraps the waitForMsg and returns succeeded or aborted, depending on the returned
    message beeing existent or None. Additionally, in the successfull case, the msg_cb, if given, will
    be called with the message and the userdata, so that a self defined method can convert message data to
    smach userdata.
    Those userdata fields have to be passed via 'output_keys'.

    If the state outcome should depend on the message content, the msg_cb can dictate the outcome:
    If msg_cb returns True, execute() will return "succeeded".
    If msg_cb returns False, execute() will return "aborted".
    If msg_cb has no return statement, execute() will act as described above.

    If thats still not enough, execute() might be overridden.

    latch: If True waitForMsg will return the last received message, so one message might be returned indefinite times.
    timeout: Seconds to wait for a message, defaults to None, disabling timeout
    output_keys: Userdata keys that the message callback needs to write to.
    """

    def __init__(self, topic, msg_type, msg_cb=None, output_keys=None, latch=False, timeout=None):
        if output_keys is None:
            output_keys = []
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'], output_keys=output_keys)
        self.latch = latch
        self.timeout = timeout
        self.mutex = threading.Lock()
        self.msg = None
        self.msg_cb = msg_cb
        self.subscriber = rospy.Subscriber(topic, msg_type, self._callback, queue_size=1)

    def __enter__(self):
        return self

    def __exit__(self, *err):
        self.subscriber.unregister()

    def _callback(self, msg):
        self.mutex.acquire()
        self.msg = msg
        self.mutex.release()

    def waitForMsg(self):
        """Await and return the message or None on timeout."""
        rospy.loginfo('Waiting for message...')
        if self.timeout is not None:
            timeout_time = rospy.Time.now() + rospy.Duration.from_sec(self.timeout)
        while self.timeout is None or rospy.Time.now() < timeout_time:
            self.mutex.acquire()
            if self.msg is not None:
                rospy.loginfo('Got message.')
                message = self.msg

                if not self.latch:
                    self.msg = None

                self.mutex.release()
                return message
            self.mutex.release()

            if self.preempt_requested():
                self.service_preempt()
                rospy.loginfo('waitForMsg is preempted!')
                return 'preempted'

            rospy.sleep(.1) # TODO: maybe convert ROSInterruptException into valid outcome

        rospy.loginfo('Timeout on waiting for message!')
        return None

    def execute(self, ud):
        """Default simplest execute(), see class description."""
        msg = self.waitForMsg()
        if msg is not None:
            if msg == 'preempted':
                return 'preempted'
            # call callback if there is one
            if self.msg_cb is not None:
                cb_result = self.msg_cb(msg, ud)
                # check if callback wants to dictate output
                if cb_result is not None:
                    if cb_result:
                        return 'succeeded'
                    else:
                        return 'aborted'
            return 'succeeded'
        else:
            return 'aborted'
{% do defined_headers.append('class_WaitForMsgState') %}{% endif %}
{% endblock class_defs %}

{% block body %}
smach.{{ parent_type }}.add('{{ name }}',
        {{ '' | indent(23, true) }}WaitForMsgState('{{ topic }}', {{ msg_type }}{% if msg_cb is defined %}, msg_cb = {% if msg_cb is expression %}{{ msg_cb|exptostr }}{% else %}'{{ msg_cb }}'{% endif %}{% endif %}{% if output_keys is defined %}, {{ render_output_keys(output_keys, indent=0) }}{% endif %}{% if latch is defined %}, latch = {{ latch }}{% endif %}{% if timeout is defined %}, timeout = {{ timeout }}{% endif %}){% if transitions is defined %},
{{ render_transitions(transitions) }}{% endif %}{% if remapping is defined %},
{{ render_remapping(remapping) }}{% endif %})
{% endblock body %}
