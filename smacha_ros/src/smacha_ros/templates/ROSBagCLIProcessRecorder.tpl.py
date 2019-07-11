{% block meta %}
name: ROSBagCLIProcessRecorder
description: >
  SMACH template that provides a ROSBagCLIProcessRecorder helper class for
  RecordROSBagState. It uses subprocess calls to the rosbag CLI (command-line
  interface) recording tool in order to circumvent threading and Python GIL
  (global interpreter lock) issues.

  See: https://answers.ros.org/question/10714/start-and-stop-rosbag-within-a-python-script/
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
{{ import_module(defined_headers, 'os') }}
{{ import_module(defined_headers, 'subprocess') }}
{# }{{ import_module(defined_headers, 'psutil') }} #}
{{ import_module(defined_headers, 'signal') }}
{{ import_module(defined_headers, 'time') }}
{% endblock imports %}

{% block class_defs %}
{% if 'class_ROSBagCLIProcessRecorder' not in defined_headers %}
class ROSBagCLIProcessRecorder(object):
    """A rosbag recorder class that uses subprocess calls to the rosbag CLI
    (command-line interface) recording tool in order to circumvent threading
    and Python GIL (global interpreter lock) issues.
    """
    def __init__(self):
        # A dict of bag recording processes indexed by bag filenames
        self._processes = dict()

    def start(self, bag_file, topics):
        """Start a rosbag recording.
        """
        try:
            if not topics:
                topics = ['-a']
            if not bag_file.endswith('.bag'):
                time_str = time.strftime('%Y-%m-%d-%H-%M-%S')
                bag_file = bag_file + '_' + time_str + '.bag'
            # cmd = ['rosbag', 'record', '-j'] + topics + ['-O', bag_file]
            cmd = ['rosbag', 'record'] + topics + ['-O', bag_file]
            rospy.loginfo('Starting rosbag CLI recording with command: \'{}\''.format(' '.join(cmd)))
            self._processes[bag_file] = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        except Exception as e:
            rospy.logerr('Unable to start recording rosbag file \'{}\' with topics {}: {}'.format(bag_file, topics, repr(e)))
            return 'aborted'

        return 'succeeded'

    def stop(self, bag_file):
        """Stop a rosbag recording.

        See: https://answers.ros.org/question/10714/start-and-stop-rosbag-within-a-python-script/
        """
        try:
            rospy.loginfo('Stopping rosbag CLI recording process for rosbag file \'{}\''.format(bag_file))

            # Kill child processes
            ps_command = subprocess.Popen('ps -o pid --ppid {} --noheaders'.format(self._processes[bag_file].pid), shell=True, stdout=subprocess.PIPE)
            ps_output = ps_command.stdout.read()
            retcode = ps_command.wait()
            assert retcode == 0, 'ps command returned {}'.format(retcode)
            for pid_str in ps_output.split("\n")[:-1]:
                os.kill(int(pid_str), signal.SIGINT)
            # Kill parent process
            os.kill(self._processes[bag_file].pid, signal.SIGINT)

        except Exception as e:
            rospy.logerr('Unable to terminate rosbag CLI recording process for rosbag file \'{}\': {}'.format(bag_file, repr(e)))
            return 'aborted'

        try:
            assert(os.path.exists(bag_file))
        except:
            rospy.logwarn('rosbag file \'{}\''.format(bag_file) +
                          'was not detected on the file system after rosbag CLI process recording stopped ' +
                          '(it may take more time for the process to terminate)!')

        return 'succeeded'

    def stop_all(self):
        """Stop all rosbag recordings.
        """
        for bag_file in list(self._processes.keys()):
            if self.stop(bag_file) != 'succeeded':
                return 'aborted'

        return 'succeeded'

{% do defined_headers.append('class_ROSBagCLIProcessRecorder') %}{% endif %}
{% endblock class_defs %}
