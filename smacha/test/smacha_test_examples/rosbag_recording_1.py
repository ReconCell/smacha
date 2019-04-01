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

from tf2_msgs.msg import TFMessage

import rosbag


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


class PublishMsgState(smach.State):
    def __init__(self, name, msg_pub_observer, action, input_keys = ['msg', 'topic'], output_keys = ['msg', 'topic'], callbacks = None):
        smach.State.__init__(self, input_keys=input_keys, output_keys=output_keys, outcomes=['succeeded', 'aborted'])

        # Save the state name
        self._name = name

        # Save the MsgPublisherObserver object reference
        self._msg_pub_observer= msg_pub_observer

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


        # Parse msg
        if self._action != 'remove_all':
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

        # Add or remove the message + publisher
        outcome = 'aborted'
        if self._action == 'add':
            if 'frame_id' in self._input_keys:
                outcome = self._msg_pub_observer.add(published_msg, topic, frame_id=userdata.frame_id)
            else:
                outcome = self._msg_pub_observer.add(published_msg, topic)
        elif self._action == 'remove':
            outcome = self._msg_pub_observer.remove(topic)
        elif self._action == 'remove_all':
            outcome = self._msg_pub_observer.remove_all()

        # Set topic output key if specified
        if self._action == 'add' and outcome == 'succeeded':
            for output_key in ['topic', 'output_topic', 'topic_output']:
                if output_key in self._output_keys:
                    setattr(userdata, output_key, topic)

        # Set msg output key if specified
        if self._action == 'add' and outcome == 'succeeded':
            for output_key in ['msg', 'output_msg', 'msg_output']:
                if output_key in self._output_keys:
                    setattr(userdata, output_key, published_msg)

        return outcome


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


class RecordROSBagState(smach.State):
    def __init__(self, name, bag_rec_observer, action, input_keys=['file', 'topics'], output_keys=[], callbacks = None):
        smach.State.__init__(self, input_keys=input_keys, output_keys=output_keys, outcomes=['succeeded', 'aborted'])

        # Save the state name
        self._name = name

        # Save the ROSBagRecorderObserver object reference
        self._bag_rec_observer= bag_rec_observer

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
            outcome = self._bag_rec_observer.start(bag_file, topics)
        elif self._action == 'stop':
            outcome = self._bag_rec_observer.stop(bag_file)
        elif self._action == 'stop_all':
            outcome = self._bag_rec_observer.stop_all()

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

    tf_msg_pub_observer = MsgPublisherObserver(sub_topic='tf')

    bag_rec_observer = ROSBagRecorderObserver()

    sm = smach.StateMachine(outcomes=['succeeded', 'aborted'])
    sm.userdata.point = Point()
    sm.userdata.topic = 'smacha/rosbag_recording_1_point'
    sm.userdata.file = '/tmp/test.bag'
    sm.userdata.topics = ['smacha/rosbag_recording_1_point']

    with sm:
        smach.StateMachine.add('PUBLISH_MSG',
                                       PublishMsgState('PUBLISH_MSG', tf_msg_pub_observer, 'add'),
                               transitions={'aborted':'aborted',
                                            'succeeded':'START_RECORDING'},
                               remapping={'msg':'point',
                                          'topic':'topic'})

        smach.StateMachine.add('START_RECORDING',
                                       RecordROSBagState('START_RECORDING', bag_rec_observer, 'start'),
                               transitions={'aborted':'aborted',
                                            'succeeded':'WAIT'},
                               remapping={'file':'file',
                                          'topics':'topics'})

        smach.StateMachine.add('WAIT',
                                       SleepState(1),
                               transitions={'succeeded':'STOP_RECORDING'})

        smach.StateMachine.add('STOP_RECORDING',
                                       RecordROSBagState('STOP_RECORDING', bag_rec_observer, 'stop_all'),
                               transitions={'aborted':'aborted',
                                            'succeeded':'UNPUBLISH_MSG'})

        smach.StateMachine.add('UNPUBLISH_MSG',
                                       PublishMsgState('UNPUBLISH_MSG', tf_msg_pub_observer, 'remove_all'),
                               transitions={'aborted':'aborted',
                                            'succeeded':'succeeded'})

    outcome = sm.execute()


if __name__ == '__main__':
    main()
