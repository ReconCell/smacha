{% block meta %}
name: MsgPublisherObserver
description:
  SMACH template that provides a MsgPublisherObserver helper class for
  PublishMsgState.
language: Python
framework: SMACH
type: None
tags: [core]
includes: []
extends:
- WaitForMsgState
variables: []
input_keys: []
output_keys: []
{% endblock meta %}

{% from "Utils.tpl.py" import import_module, from_import %}

{% extends "WaitForMsgState.tpl.py" %}

{% block imports %}
{{ super() }}
{{ from_import(defined_headers, 'tf2_msgs.msg', 'TFMessage') }}
{% endblock imports %}

{% block class_defs %}
{{ super() }}
{% if 'class_MsgPublisherObserver' not in defined_headers %}
class MsgPublisherObserver(object):
    """
    Subscribes to a topic and maintains dicts of messages & publishers
    that are updated with the topic subscription callback.
    """
    def __init__(self, sub_topic='/tf'):

        # Save sub_topic
        self._sub_topic = sub_topic

        # A subject topic to be subscribed to in order to synchronize message
        # publication.
        self._sub = None

        # A dict of messages
        self._msgs = dict()

        # A dict of message publishers
        self._pubs = dict()

    def _pub_cb(self, data):
        # Publish messages
        if self._msgs:
            for topic, msg in self._msgs.items():
                try:
                    # Update the timestamp
                    msg.header.stamp = rospy.Time.now()
                except Exception as e:
                    rospy.logwarn('Failed to update timestamp for topic \'{}\': {}'.format(topic, repr(e)))
                    pass

                try:
                    # Publish
                    self._pubs[topic].publish(msg)
                except Exception as e:
                    rospy.logwarn('Failed to publish message on topic \'{}\': {}'.format(topic, repr(e)))

    def add(self, msg, topic, frame_id=None):
        # Subscribe to the subject topic if this has not already been done
        if not self._sub:
            # Try detecting the message type/class of the sub_topic
            # See: https://schulz-m.github.io/2016/07/18/rospy-subscribe-to-any-msg-type/
            try:
                with (WaitForMsgState)(self._sub_topic, rospy.AnyMsg, latch=True) as wait_for_any_msg:
                    any_msg = wait_for_any_msg.waitForMsg()
                    msg_type = any_msg._connection_header['type']
                    rospy.loginfo('Message type for topic {} detected as {}'.format(topic, msg_type))
                    msg_class = roslib.message.get_message_class(msg_type)
            except Exception as e:
                self._bags[bag_file].close()
                rospy.logerr('Failed to detect message type/class for topic {}: {}'.format(topic, repr(e)))
                return 'aborted'

            # Set up the subscriber
            try:
                self._sub = rospy.Subscriber(self._sub_topic, msg_class, self._pub_cb)
            except Exception as e:
                raise ValueError('Failed to subscribe to TFMessage topic: {}'.format(repr(e)))

        # Add message with specified frame_id
        try:
            assert frame_id
            self._msgs[topic] = msg
            self._msgs[topic].header.frame_id = frame_id
        except:
            # Add message with frame_id unspecified
            try:
                self._msgs[topic] = msg
            except Exception as e:
                rospy.logwarn('Failed to add message for publication on topic \'{}\': {}'.format(topic, repr(e)))
                return 'aborted'

        # Add publisher
        try:
            self._pubs[topic] = rospy.Publisher(topic, type(msg))
        except Exception as e:
            self._msgs.pop(topic,0)
            rospy.logwarn('Failed to add publisher for topic \'{}\': {}'.format(topic, repr(e)))
            return 'aborted'
        return 'succeeded'

    def remove(self, topic):
        try:
            # Remove message + publisher
            self._msgs.pop(topic, 0)
            self._pubs.pop(topic, 0)
        except Exception as e:
            rospy.logwarn('Failed to remove publisher for topic \'{}\': {}'.format(topic, repr(e)))
            return 'aborted'
        return 'succeeded'

    def remove_all(self):
        try:
            self._msgs.clear()
            self._pubs.clear()
        except Exception as e:
            rospy.logwarn('Failed to remove all publishers: {}'.format(repr(e)))
            return 'aborted'
        return 'succeeded'
{% do defined_headers.append('class_MsgPublisherObserver') %}{% endif %}
{% endblock class_defs %}

{% block header %}
{% endblock header %}

{% block body %}
{% endblock body %}
