{% block meta %}
name: ROSBagAPIThreadRecorder
description: >
  SMACH template that provides a ROSBagAPIThreadRecorder helper class for
  RecordROSBagState. It uses the rosbag API (application programming interface)
  as well as the threading library in order to manage multiple recording
  threads. NOTE: this means that this recorder may have issues with the Python
  GIL (global interpreter lock) when other threads (e.g. MoveIt! commands)
  block execution.

  Based in part on code from:
  https://github.com/francisc0garcia/sensor_board/blob/master/src/classes/bags/recorder.py
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
{{ import_module(defined_headers, 'rosbag') }}
{{ import_module(defined_headers, 'roslib') }}
{{ import_module(defined_headers, 'rosgraph') }}
{{ import_module(defined_headers, 'threading') }}
{% if 'import_Queue' not in defined_headers %}
try:
    from queue import Queue
except ImportError:
    from Queue import Queue
{% do defined_headers.append('import_Queue') %}
{% endif %}
{% endblock imports %}

{% block class_defs %}
{% if 'class_ROSBagAPIThreadRecorder' not in defined_headers %}
class ROSBagAPIThreadRecorder(object):
    """A rosbag recorder class that uses the rosbag API (application
    programming interface) as well as the threading library in order to manage
    multiple recording threads. NOTE: this means that this recorder may have
    issues with the Python GIL (global interpreter lock) when other threads (e.g.
    MoveIt! commands) block execution.
    """
    def __init__(self):
        # Get a reference to the ROS master
        self._master = rosgraph.Master('rosbag_recorder_observer')

        # A dict of bag master check threads indexed by bag filenames
        self._master_check_threads = dict()

        # The rate at which to poll the ROS master for new topics
        self._master_check_interval = 0.1

        # A dict of rosbags indexed by filenames
        self._bags = dict()

        # A dict of bag writing threads indexed by bag filenames
        self._write_threads = dict()

        # A dict of bag writing queues indexed by bag filenames
        self._write_queues = dict()

        # A dict of bag writing stop flags indexed by bag filenames
        self._stop_flags = dict()

        # A dict of bag thread stop conditions indexed by bag filenames
        self._stop_conditions = dict()

        # A dict of bag file locks indexed by bag filenames
        self._bag_locks = dict()

        # A dict of dicts of subscribers indexed by bag_files and topics
        # respectively
        self._bag_subs = dict()

        # Length of timeout (in seconds), as well as sleep rate, for waiting
        # for the threads to finish writing before forcibly closing a bag.
        self._bag_close_timeout = 10.0
        self._bag_close_sleep_rate = 100.0

    def _write_cb(self, msg, args):
        bag_file = args[0]
        topic = args[1]
        msg_class = args[2]
        try:
            self._write_queues[bag_file].put((topic, msg, rospy.get_rostime()))
        except Exception as e:
            rospy.logwarn('Failed to write message of type {} from topic {} to rosbag {}: {}'.format(msg_class, topic, bag_file, repr(e)))
            pass

    def _run_master_check_thread(self, bag_file, topics):
        # Set up an observer loop
        try:
            while not self._stop_flags[bag_file]:
                # Get a list of topics currently being published
                currently_published_topics = []
                try:
                    currently_published_topics = self._master.getPublishedTopics('')
                except Exception as e:
                    # TODO: Allow this warning to be included if a
                    # debug/verbosity flag is passed to the state.
                    # rospy.logwarn('Failed to get list of currently published topics from ROS master: {}'.format(repr(e)))
                    pass

                # Check for new topics
                for topic, msg_type in currently_published_topics:
                    # If the topic has previously been subscribed to for this
                    # bag_file, or is not listed as a topic for this bag_file,
                    # skip it.
                    if topic in list(self._bag_subs[bag_file].keys()) or (topic not in topics and topic.strip('/') not in topics):
                        continue

                    # Subscribe to the topic
                    try:
                        msg_class = roslib.message.get_message_class(msg_type)
                        self._bag_subs[bag_file][topic] = rospy.Subscriber(topic, msg_class, self._write_cb, (bag_file, topic, msg_class))
                    except Exception as e:
                        self._unsubscribe_bag_topics(bag_file)
                        self._close_bag(bag_file)
                        raise ValueError('Failed to subscribe to topic {}: {}'.format(topic, repr(e)))

                # Wait a while
                self._stop_conditions[bag_file].acquire()
                self._stop_conditions[bag_file].wait(self._master_check_interval)

        except Exception as e:
            rospy.logerr('Error when recording rosbag file {}: {}'.format(bag_file, repr(e)))

        # Unsubscribe from topics and close bag
        self._unsubscribe_bag_topics(bag_file)
        self._close_bag(bag_file)

    def _unsubscribe_bag_topics(self, bag_file):
        for _, sub in self._bag_subs[bag_file].items():
            try:
                sub.unregister()
            except Exception as e:
                rospy.logerr('Failed to unregister topic subscriber {} while stopping rosbag recording with filename \'{}\': {}'.format(sub, bag_file, repr(e)))
                raise
        del self._bag_subs[bag_file]

    def _close_bag(self, bag_file):
        try:
            with self._bag_locks[bag_file]:
                self._bags[bag_file].close()
        except Exception as e:
            rospy.logerr('Failed to close rosbag with filename \'{}\': {}'.format(bag_file, repr(e)))
            raise
        del self._bags[bag_file]

    def _run_write_thread(self, bag_file):
        try:
            while not self._stop_flags[bag_file]:
                # Wait for a message
                item = self._write_queues[bag_file].get()

                if item == self:
                    continue

                topic, msg, t = item

                # Write to the bag
                with self._bag_locks[bag_file]:
                    self._bags[bag_file].write(topic, msg, t)

        except Exception as e:
            rospy.logerr('Error when writing to rosbag file {}: {}'.format(bag_file, repr(e)))

    def start(self, bag_file, topics):
        """Start a rosbag recording.
        """
        # Open the bag file for writing
        try:
            assert(bag_file not in self._bags.keys())
            self._bags[bag_file] = rosbag.Bag(bag_file, 'w')
            self._bag_subs[bag_file] = dict()
        except Exception as e:
            rospy.logerr('Failed to start rosbag recording with filename \'{}\': {}'.format(bag_file, repr(e)))
            return 'aborted'

        # Set up the bag writing queue
        self._write_queues[bag_file] = Queue()

        # Set the bag thread lock, write stopping flag, and thread stopping conditions
        self._bag_locks[bag_file] = threading.Lock()
        self._stop_flags[bag_file] = False
        self._stop_conditions[bag_file] = threading.Condition()

        # Spin up the master check and bag writing threads
        self._master_check_threads[bag_file] = threading.Thread(target=self._run_master_check_thread, args=[bag_file, topics])
        self._write_threads[bag_file] = threading.Thread(target=self._run_write_thread, args=[bag_file])
        self._master_check_threads[bag_file].start()
        self._write_threads[bag_file].start()

        return 'succeeded'

    def stop(self, bag_file):
        """Stop a rosbag recording.
        """
        # Signal threads to stop bag recording
        with self._stop_conditions[bag_file]:
            self._stop_flags[bag_file] = True
            self._stop_conditions[bag_file].notify_all()

        # Signal the bag write thread to stop writing
        self._write_queues[bag_file].put(self)

        # Wait for the bag to be closed
        t = rospy.get_time()
        r = rospy.Rate(self._bag_close_sleep_rate)
        while bag_file in list(self._bags.keys()):
            if rospy.get_time() - t < self._bag_close_timeout:
                r.sleep()
            else:
                break
        else:
            return 'succeeded'

        # If the bag is still open, issue a warning and attempt forced closure.
        rospy.logwarn('Warning: timeout exceeded for stopping writing to rosbag file {}. Attempting forced stop...'.format(bag_file))
        try:
            self._unsubscribe_bag_topics(bag_file)
            self._close_bag(bag_file)
        except Exception as e:
            rospy.logerr('Error during forced stop of writing to rosbag file {}: {}'.format(bag_file, repr(e)))
            return 'aborted'

        return 'succeeded'

    def stop_all(self):
        """Stop all rosbag recordings.
        """
        # Stop all current recordings
        for bag_file in list(self._bags.keys()):
            if self.stop(bag_file) != 'succeeded':
                return 'aborted'

        return 'succeeded'

{% do defined_headers.append('class_ROSBagAPIThreadRecorder') %}{% endif %}
{% endblock class_defs %}
