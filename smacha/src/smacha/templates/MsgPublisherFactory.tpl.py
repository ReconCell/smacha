{% block meta %}
name: MsgPublisherFactory
description:
  SMACH template that provides a MsgPublisherFactory helper class for
  PublishMsgState.
language: Python
framework: SMACH
type: None
tags: [core]
includes: []
extends: []
variables: []
input_keys: []
output_keys: []
{% endblock meta %}

{% from "Utils.tpl.py" import import_module, from_import %}

{% block imports %}
{{ import_module(defined_headers, 'roslib') }}
{{ from_import(defined_headers, 'tf2_msgs.msg', 'TFMessage') }}
{% endblock imports %}

{% block class_defs %}
{% if 'class_MsgPublisherFactory' not in defined_headers %}
class MsgPublisherFactory(object):
    def __init__(self, sub_topic='/tf'):

        # Save sub_topic
        self._sub_topic = sub_topic

        # A subscriber, to time publication w.r.t. a tf topic
        #
        # NOTE: This was previously set to use a 'rospy.AnyMsg' topic,
        #       but this appeared to break TF2 when attempting to listen
        #       for transforms.
        try:
            self._sub = rospy.Subscriber(self._sub_topic, TFMessage, self._pub_cb)
        except Exception as e:
            raise ValueError('Failed to subscribe to TFMessage topic: {}'.format(repr(e)))

        # A dict of messages
        self._msgs = dict()

        # A dict of message publishers
        self._pubs = dict()

    def add(self, msg, topic, frame_id=None):
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
{% do defined_headers.append('class_MsgPublisherFactory') %}{% endif %}
{% endblock class_defs %}
