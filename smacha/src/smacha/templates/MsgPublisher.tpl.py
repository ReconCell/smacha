{% block meta %}
name: MsgPublisher
description:
  SMACH template that provides a MsgPublisher helper class for
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
{{ import_module(defined_headers, 'rospy') }}
{{ import_module(defined_headers, 'threading') }}
{% endblock imports %}

{% block class_defs %}
{% if 'class_MsgPublisher' not in defined_headers %}
class MsgPublisher(object):
    """
    """
    def __init__(self):
        # A dict of message publishers indexed by topic
        self._pubs = dict()

        # A dict of messages indexed by topic
        self._msgs = dict()

        # A dict of callbacks indexed by topic
        self._callbacks = dict()

        # A dict of message publication rates indexed by topic
        self._pub_rates = dict()

        # A dict of message publisher threads indexed by topic
        self._pub_threads = dict()

        # A dict of message publisher stop flags indexed by topic
        self._stop_flags = dict()

        # Length of timeout (in seconds) for waiting for the threads to finish
        # publishing before forcibly unpublishing.
        self._unpublish_timeout = 10.0

    def _run_pub_thread(self, topic):
        r = rospy.Rate(self._pub_rates[topic])
        while not self._stop_flags[topic]:
            # Apply callback to message
            if self._callbacks[topic]:
                try:
                    self._msgs[topic] = self._callbacks[topic](self._msgs[topic])
                except Exception as e:
                    rospy.logerr('Error when applying callback to message being published on topic {}: {}'.format(topic, repr(e)))
            # Publish message
            try:
                self._pubs[topic].publish(self._msgs[topic])
            except Exception as e:
                rospy.logerr('Error while publishing to topic {}: {}'.format(topic, repr(e)))
            r.sleep()

        self._unpublish(topic)

    def _unpublish(self, topic):
        try:
            self._pubs[topic].unregister()
        except Exception as e:
            rospy.logerr('Failed to unregister publisher of topic {}: {}'.format(topic, repr(e)))
            raise
        del self._pubs[topic]
        del self._msgs[topic]
        del self._callbacks[topic]
        del self._pub_rates[topic]

    def start(self, msg, topic, rate, frame_id=None, callback=None):
        # Set the message publisher stopping flag
        self._stop_flags[topic] = False

        # Save the message
        self._msgs[topic] = msg

        # Save the message publication rate
        self._pub_rates[topic] = rate

        # Use frame_id if specified
        if frame_id:
            try:
                assert(isinstance(frame_id, str))
                self._msgs[topic].header.frame_id = frame_id
            except:
                rospy.logwarn('Failed to add specified frame_id {} to message for publication on topic {}: {}'.format(frame_id, topic, repr(e)))

        # Use callback if specified
        if callback:
            try:
                assert(callable(callback))
                self._callbacks[topic] = callback
            except:
                rospy.logwarn('Failed to add specified callback {} to publisher of topic {}: {}'.format(callback, topic, repr(e)))
                self._callbacks[topic] = None
        else:
            self._callbacks[topic] = None

        # Add publisher
        try:
            self._pubs[topic] = rospy.Publisher(topic, type(self._msgs[topic]))
        except Exception as e:
            del self._pub_rates[topic]
            self._msgs[topic]
            rospy.logwarn('Failed to add publisher for topic {}: {}'.format(topic, repr(e)))
            return 'aborted'

        # Spin up the message publication thread
        self._pub_threads[topic] = threading.Thread(target=self._run_pub_thread, args=[topic])
        self._pub_threads[topic].start()

        return 'succeeded'

    def stop(self, topic):
        # Signal thread to stop publishing
        self._stop_flags[topic] = True

        # Wait for the topic to be unpublished
        t = rospy.get_time()
        r = rospy.Rate(self._pub_rates[topic])
        while topic in list(self._pubs.keys()):
            if rospy.get_time() - t < self._unpublish_timeout:
                r.sleep()
            else:
                break
        else:
            return 'succeeded'

        # If the publisher is still running, issue a warning and attempt forced unpublish.
        rospy.logwarn('Warning: timeout exceeded for stopping publisher thread for topic {}. Attempting forced stop...'.format(topic))
        try:
            self._unpublish(topic)
        except Exception as e:
            rospy.logerr('Error during forced stop of publisher of topic {}: {}'.format(topic, repr(e)))
            return 'aborted'

        return 'succeeded'

    def stop_all(self):
        # Stop all current publishers
        for topic in self._pubs.keys():
            if self.stop(topic) != 'succeeded':
                return 'aborted'

        return 'succeeded'

{% do defined_headers.append('class_MsgPublisher') %}{% endif %}
{% endblock class_defs %}
