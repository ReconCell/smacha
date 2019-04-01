{% block meta %}
name: ROSBagRecorderObserver
description:
  SMACH template that provides a ROSBagRecorderObserver helper class for
  RecordROSBagState.
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
{{ import_module(defined_headers, 'rospy') }}
{{ import_module(defined_headers, 'rosbag') }}
{% endblock imports %}

{% block class_defs %}
{{ super() }}
{% if 'class_ROSBagRecorderObserver' not in defined_headers %}
class ROSBagRecorderObserver(object):
    """
    Subscribes to topics and maintains a dicts of currently open rosbag
    recordings & topic subscribers that are updated with the topic subscription
    callbacks.
    """
    def __init__(self):
        # A dict of rosbags indexed by filename
        self._bags = dict()

        # A dict of dicts of subscribers indexed by bag_file and topic
        # respectively
        self._bag_subs = dict()

    def _write_cb(self, data, bag_file, topic, msg_class):
        try:
            self._bags[bag_file].write(topic, data)
        except Exception as e:
            rospy.logwarn('Failed to write data of type {} from topic {} to rosbag {}: {}'.format(msg_class, topic, bag_file, repr(e)))
            pass

    def start(self, bag_file, topics):
        # Open the bag file for writing
        try:
            assert(bag_file not in self._bags.keys())
            self._bags[bag_file] = rosbag.Bag(bag_file, 'w')
            self._bag_subs[bag_file] = dict()
        except Exception as e:
            rospy.logerr('Failed to start rosbag recording with filename \'{}\': {}'.format(bag_file, repr(e)))
            return 'aborted'

        # Subscribe to the topics
        for topic in topics:
            if topic not in self._bag_subs[bag_file].keys():
                # Try detecting the message type/class if necessary.
                # See: https://schulz-m.github.io/2016/07/18/rospy-subscribe-to-any-msg-type/
                try:
                    with (WaitForMsgState)(topic, rospy.AnyMsg, latch=True) as wait_for_any_msg:
                        any_msg = wait_for_any_msg.waitForMsg()
                        msg_type = any_msg._connection_header['type']
                        rospy.loginfo('Message type for topic {} detected as {}'.format(topic, msg_type))
                        msg_class = roslib.message.get_message_class(msg_type)
                except Exception as e:
                    self._bags[bag_file].close()
                    rospy.logerr('Failed to detect message type/class for topic {}: {}'.format(topic, repr(e)))
                    return 'aborted'

            # Subscribe to the topic
            try:
                self._bag_subs[bag_file][topic] = rospy.Subscriber(topic, msg_class, lambda data: self._write_cb(data, bag_file, topic, msg_class))
            except Exception as e:
                self._bags[bag_file].close()
                raise ValueError('Failed to subscribe to TFMessage topic: {}'.format(repr(e)))

        return 'succeeded'

    def stop(self, bag_file):
        # Stop bag recording
        self._bags[bag_file].close()

        # Unsubscribe from bag topics
        for _, sub in self._bag_subs[bag_file].items():
            sub.unregister()
        del self._bag_subs[bag_file]

        return 'succeeded'

    def stop_all(self):
        # Stop all current recordings
        for bag_file in self._bags.keys():
            self.stop(bag_file)

        return 'succeeded'

{% do defined_headers.append('class_ROSBagRecorderObserver') %}{% endif %}
{% endblock class_defs %}

{% block header %}
{% endblock header %}

{% block body %}
{% endblock body %}
