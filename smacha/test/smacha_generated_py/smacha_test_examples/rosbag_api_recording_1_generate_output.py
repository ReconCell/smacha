#!/usr/bin/env python




import roslib
import rospy
import smach
import smach_ros
from geometry_msgs.msg import Point
from geometry_msgs.msg import Point32
from geometry_msgs.msg import PointStamped







from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped

from geometry_msgs.msg import Quaternion









from geometry_msgs.msg import PoseArray









from sensor_msgs.msg import PointCloud

from sensor_msgs import point_cloud2 as pc2








from sensor_msgs.msg import PointCloud2
from sensor_msgs.point_cloud2 import create_cloud_xyz32








import threading

import rosbag

import rosgraph


try:
    from queue import Queue
except ImportError:
    from Queue import Queue



def parse_pointstamped(point_input):
    """
    Parse point_input into PointStamped.
    """
    try:
        assert isinstance(point_input, PointStamped)
        return point_input
    except:
        pass
    try:
        assert isinstance(point_input, Point)
        point = PointStamped(point = point_input)
        point.header.stamp = rospy.Time.now()
        return point
    except:
        pass
    try:
        assert isinstance(point_input, Point32)
        point = PointStamped(point = Point(x=point_input.x, y=point_input.y, z=point_input.z))
        point.header.stamp = rospy.Time.now()
        return point
    except:
        pass
    try:
        point = point_input
        point = PointStamped(point = Point(x=point[0], y=point[1], z=point[2]))
        point.header.stamp = rospy.Time.now()
        return point
    except Exception as e:
        raise ValueError('Point not properly specified (should be Point, PointStamped or [3] list type)!')









def parse_posestamped(pose_input):
    """
    Parse pose_input into PoseStamped.
    """
    try:
        assert isinstance(pose_input, PoseStamped)
        return pose_input
    except:
        pass
    try:
        assert isinstance(pose_input, Pose)
        pose = PoseStamped(pose = pose_input)
        pose.header.stamp = rospy.Time.now()
        return pose
    except:
        pass
    try:
        pose = pose_input
        position = Point(x=pose_input[0][0], y=pose_input[0][1], z=pose_input[0][2])
        orientation = Quaternion(x=pose_input[1][0], y=pose_input[1][1], z=pose_input[1][2], w=pose_input[1][3])
        pose = PoseStamped(pose = Pose(position=position, orientation=orientation))
        pose.header.stamp = rospy.Time.now()
        return pose
    except Exception as e:
        raise ValueError('Pose not properly specified (should be Pose, PoseStamped or [[3],[4]] list)!')









def parse_posearray(posearray_input):
    """
    Parse posearray_input into a PoseArray.
    """
    try:
        assert isinstance(posearray_input, PoseArray)
        return posearray_input
    except:
        pass
    try:
        assert isinstance(posearray_input, list)
        posearray = PoseArray()
        for pose in posearray_input:
            try:
                assert isinstance(pose, Pose)
                posearray.poses.append(pose)
                continue
            except:
                pass
            try:
                assert isinstance(pose, PoseStamped)
                posearray.poses.append(pose.pose)
                continue
            except:
                pass
            try:
                position = Point(x=pose[0][0], y=pose[0][1], z=pose[0][2])
                orientation = Quaternion(x=pose[1][0], y=pose[1][1], z=pose[1][2], w=pose[1][3])
                pose = Pose(position=position, orientation=orientation)
                posearray.poses.append(pose)
                continue
            except Exception as e:
                raise ValueError('Pose in pose array input not properly specified (should be Pose, PoseStamped or [[3],[4]] list)!')
        posearray.header.stamp = rospy.Time.now()
        return posearray
    except Exception as e:
        raise ValueError('Pose array not properly specified (should be PoseArray or list of Pose, PoseStamped or [[3],[4]] list types)!')









def parse_pointcloud(pointcloud_input):
    """
    Parse pointcloud_input into PointCloud.
    """
    try:
        assert isinstance(pointcloud_input, PointCloud)
        return pointcloud_input
    except:
        pass
    try:
        points = pc2.read_points(pointcloud_input, skip_nans=True, field_names=('x', 'y', 'z'))
        return PointCloud(points = map(lambda point: Point32(*point), points))
    except Exception as e:
        raise ValueError('Point cloud not properly specified (should be PointCloud or PointCloud2 type): ' + repr(e))









def parse_pointcloud2(pointcloud_input):
    """
    Parse pointcloud_input into PointCloud2.
    """
    try:
        assert isinstance(pointcloud_input, PointCloud2)
        return pointcloud_input
    except:
        pass
    try:
        points = [[point.x, point.y, point.z] for point in pointcloud_input.points]
        pointcloud2 = create_cloud_xyz32(header=pointcloud_input.header, points=points)
        return pointcloud2
    except:
        raise ValueError('Point cloud not properly specified (should be PointCloud or PointCloud2 type)!')



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
















class PublishMsgState(smach.State):
    def __init__(self, name, msg_publisher, action, input_keys = ['msg', 'topic', 'rate'], output_keys = ['msg', 'topic'], callbacks = None):
        smach.State.__init__(self, input_keys=input_keys, output_keys=output_keys, outcomes=['succeeded', 'aborted'])

        # Save the state name
        self._name = name

        # Save the MsgPublisherObserver object reference
        self._msg_publisher = msg_publisher

        # Save the action
        self._action = action

        # Set up dict of parsing functions for certain message types/classes.
        self._msg_parsers = {"<class 'geometry_msgs.msg._Point.Point'>": parse_pointstamped,
                             "<class 'geometry_msgs.msg._PointStamped.PointStamped'>": parse_pointstamped,
                             "<class 'geometry_msgs.msg._Pose.Pose'>": parse_posestamped,
                             "<class 'geometry_msgs.msg._PoseStamped.PoseStamped'>": parse_posestamped,
                             "<class 'geometry_msgs.msg._PoseArray.PoseArray'>": parse_posearray,
                             "<class 'sensor_msgs.msg._PointCloud.PointCloud'>": parse_pointcloud,
                             "<class 'sensor_msgs.msg._PointCloud2.PointCloud2'>": parse_pointcloud2}

        
        self._cbs = []

        if callbacks:
            for cb in sorted(callbacks):
                if cb in globals():
                    self._cbs.append(globals()[cb])
                elif cb in locals():
                    self._cbs.append(locals()[cb])
                elif cb in dir(self):
                    self._cbs.append(getattr(self, cb))

        self._cb_input_keys = []
        self._cb_output_keys = []
        self._cb_outcomes = []

        for cb in self._cbs:
            if cb and smach.has_smach_interface(cb):
                self._cb_input_keys.append(cb.get_registered_input_keys())
                self._cb_output_keys.append(cb.get_registered_output_keys())
                self._cb_outcomes.append(cb.get_registered_outcomes())

                self.register_input_keys(self._cb_input_keys[-1])
                self.register_output_keys(self._cb_output_keys[-1])
                self.register_outcomes(self._cb_outcomes[-1])


    def _parse_msg(self, msg, msg_type=None):
        # First try using a known parser for a specified msg_type.
        try:
            assert msg_type
            msg_class = str(roslib.message.get_message_class(msg_type))
            published_msg = self._msg_parsers[msg_class](msg)
            return published_msg
        except:
            pass

        # Next, try to select a known parser by checking the type of message.
        try:
            msg_class = str(type(msg))
            published_msg = self._msg_parsers[msg_class](msg)
            return published_msg
        except:
            pass

        # Next, try each message type parser in succession and see if something sticks.
        for _, parser in self._msg_parsers.items():
            try:
                published_msg = parser(msg)
                return published_msg
            except:
                pass

        # Finally, if none of the above stuck, just return the original message.
        return msg

    def execute(self, userdata):

        
        # Call callbacks
        for (cb, ik, ok) in zip(self._cbs,
                                self._cb_input_keys,
                                self._cb_output_keys):

            # Call callback with limited userdata
            try:
                cb_outcome = cb(self, smach.Remapper(userdata,ik,ok,{}))
            except:
                cb_outcome = cb(smach.Remapper(userdata,ik,ok,{}))


        # Start or stop the message publisher
        outcome = 'aborted'
        if self._action == 'start':
            # Parse msg
            try:
                if 'msg_type' in self._input_keys:
                    published_msg = self._parse_msg(userdata.msg, msg_type=userdata.msg_type)
                else:
                    published_msg = self._parse_msg(userdata.msg)
            except Exception as e:
                rospy.logerr('Failed to parse message: '.format(repr(e)))
                return 'aborted'

            # Get topic if it's specified as an input key
            if 'topic' in self._input_keys:
                topic = userdata.topic
            # Otherwise, construct it from the state name
            else:
                topic = 'smacha/' + self._name.lower()

            # Get rate if it's specified as an input key
            if 'rate' in self._input_keys:
                rate = userdata.rate
            else:
                rate = 100.0

            # Get callback if it's specified as an input key
            if 'callback' in self._input_keys:
                callback = userdata.callback
            else:
                callback = ''

            # Get frame_id if it's specified as an input key
            if 'frame_id' in self._input_keys:
                frame_id = userdata.frame_id
            else:
                frame_id = ''

            # Start the publisher
            outcome = self._msg_publisher.start(published_msg, topic, rate, frame_id=frame_id, callback=callback)

        elif self._action == 'stop':
            outcome = self._msg_publisher.stop(topic)

        elif self._action == 'stop_all':
            outcome = self._msg_publisher.stop_all()

        # Set topic output key if specified
        if self._action == 'start' and outcome == 'succeeded':
            for output_key in ['topic', 'output_topic', 'topic_output']:
                if output_key in self._output_keys:
                    setattr(userdata, output_key, topic)

        # Set msg output key if specified
        if self._action == 'start' and outcome == 'succeeded':
            for output_key in ['msg', 'output_msg', 'msg_output']:
                if output_key in self._output_keys:
                    setattr(userdata, output_key, published_msg)

        return outcome

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
















class RecordROSBagState(smach.State):
    def __init__(self, name, bag_recorder, action, input_keys=['file', 'topics'], output_keys=[], callbacks = None):
        smach.State.__init__(self, input_keys=input_keys, output_keys=output_keys, outcomes=['succeeded', 'aborted'])

        # Save the state name
        self._name = name

        # Save the ROSBagRecorder object reference
        self._bag_recorder= bag_recorder

        # Save the action
        self._action = action

        
        self._cbs = []

        if callbacks:
            for cb in sorted(callbacks):
                if cb in globals():
                    self._cbs.append(globals()[cb])
                elif cb in locals():
                    self._cbs.append(locals()[cb])
                elif cb in dir(self):
                    self._cbs.append(getattr(self, cb))

        self._cb_input_keys = []
        self._cb_output_keys = []
        self._cb_outcomes = []

        for cb in self._cbs:
            if cb and smach.has_smach_interface(cb):
                self._cb_input_keys.append(cb.get_registered_input_keys())
                self._cb_output_keys.append(cb.get_registered_output_keys())
                self._cb_outcomes.append(cb.get_registered_outcomes())

                self.register_input_keys(self._cb_input_keys[-1])
                self.register_output_keys(self._cb_output_keys[-1])
                self.register_outcomes(self._cb_outcomes[-1])


    def execute(self, userdata):

        
        # Call callbacks
        for (cb, ik, ok) in zip(self._cbs,
                                self._cb_input_keys,
                                self._cb_output_keys):

            # Call callback with limited userdata
            try:
                cb_outcome = cb(self, smach.Remapper(userdata,ik,ok,{}))
            except:
                cb_outcome = cb(smach.Remapper(userdata,ik,ok,{}))


        # Get filename from userdata
        try:
            bag_file = userdata.file
            assert(isinstance(bag_file, str))
        except Exception as e:
            rospy.logerr('The rosbag filename must be specified as a userdata input key: {}'.format(repr(e)))
            return 'aborted'

        # Get topic names from userdata
        try:
            topics = userdata.topics
            assert(not any(not isinstance(x, str) for x in topics))
        except Exception as e:
            rospy.logerr('Topic names must be specified as a userdata input key: {}'.format(repr(e)))
            return 'aborted'

        # Start or stop recording
        outcome = 'aborted'
        if self._action == 'start' or self._action == 'record':
            outcome = self._bag_recorder.start(bag_file, topics)
        elif self._action == 'stop':
            outcome = self._bag_recorder.stop(bag_file)
        elif self._action == 'stop_all':
            outcome = self._bag_recorder.stop_all()

        return outcome

class SleepState(smach.State):
    def __init__(self, time, input_keys = [], output_keys = [], callbacks = [], outcomes=['succeeded']):
        smach.State.__init__(self, input_keys=input_keys, output_keys=output_keys, outcomes=outcomes)

        self._time = time

    def execute(self, userdata):

        rospy.sleep(self._time)

        return 'succeeded'









def main():
    rospy.init_node('sm')

    msg_publisher = MsgPublisher()

    bag_recorder = ROSBagAPIThreadRecorder()



    sm = smach.StateMachine(outcomes=['succeeded', 'aborted'])


    sm.userdata.rate = 100.0

    sm.userdata.file = ''

    sm.userdata.topics = ''

    sm.userdata.rate = 100.0

    sm.userdata.topic = ''




    sm.userdata.point = Point()

    sm.userdata.rate = 100.0

    sm.userdata.topic = 'smacha/rosbag_api_recording_1_point'

    sm.userdata.file = '/tmp/rosbag_api_recording_1.bag'

    sm.userdata.topics = ['smacha/rosbag_api_recording_1_point']

    with sm:


        smach.StateMachine.add('PUBLISH_MSG',
                                       PublishMsgState('PUBLISH_MSG', msg_publisher, 'start'),
                               transitions={'aborted':'aborted',
                                            'succeeded':'START_RECORDING'},
                               remapping={'msg':'point',
                                          'rate':'rate',
                                          'topic':'topic'})

        smach.StateMachine.add('START_RECORDING',
                                       RecordROSBagState('START_RECORDING', bag_recorder, 'start'),
                               transitions={'aborted':'aborted',
                                            'succeeded':'WAIT'},
                               remapping={'file':'file',
                                          'topics':'topics'})

        smach.StateMachine.add('WAIT',
                                       SleepState(5),
                               transitions={'succeeded':'STOP_RECORDING'})

        smach.StateMachine.add('STOP_RECORDING',
                                       RecordROSBagState('STOP_RECORDING', bag_recorder, 'stop_all'),
                               transitions={'aborted':'aborted',
                                            'succeeded':'UNPUBLISH_MSG'})

        smach.StateMachine.add('UNPUBLISH_MSG',
                                       PublishMsgState('UNPUBLISH_MSG', msg_publisher, 'stop_all'),
                               transitions={'aborted':'aborted',
                                            'succeeded':'succeeded'})



        





    

    outcome = sm.execute()





    



if __name__ == '__main__':
    main()