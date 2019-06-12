{% block meta %}
name: DeleteFileState
description:
  SMACH state that deletes specified files from the file system.
language: Python
framework: SMACH
type: State
tags: [core]
includes: []
extends:
- State
variables:
- callbacks:
    description:
      Either callback function names or backtick-wrapped lambda functions for,
      e.g. modifying the filenames.
    type: dict of str
- - input_keys:
      description:
          The userdata input keys, e.g. needed by the optionally specified callbacks.
      type: list of str
- - output_keys:
      description:
          The userdata output keys, e.g. needed by the optionally specified
          callbacks.
      type: list of str
input_keys:
- file:
    description:
      The file, files or file specification for deletion.
      A deletion loop is called on glob.glob(userdata.file).
    type: str
output_keys: []
outcomes:
- succeeded
- aborted
{% endblock meta %}

{% from "Utils.tpl.py" import import_module, render_init_callbacks, render_execute_callbacks %}

{% extends "State.tpl.py" %}

{% block imports %}
{{ super() }}
{{ import_module(defined_headers, 'rospy') }}
{{ import_module(defined_headers, 'os') }}
{{ import_module(defined_headers, 'glob') }}
{% endblock imports %}

{% block class_defs %}
{{ super() }}
{% if 'class_DeleteFileState' not in defined_headers %}
class DeleteFileState(smach.State):
    """
    This state deletes files.
    """
    def __init__(self, input_keys = ['file'], output_keys = [], callbacks=None):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'], input_keys=input_keys, output_keys=output_keys)

        {{ render_init_callbacks() }}

    def execute(self, userdata):

        {{ render_execute_callbacks() }}

        try:
            rospy.loginfo("Deleting '{}'".format(userdata.file)) 
            os.remove(userdata.file)
        except:
            try:
                for f in glob.glob(userdata.file):
                    rospy.loginfo("Deleting '{}'".format(f)) 
                    os.remove(f)
            except Exception as e:
                rospy.logerr("Failed to delete file(s) with filespec: '{}'!".format(userdata.file)) 
                return 'aborted'

        return 'succeeded'
{% do defined_headers.append('class_DeleteFileState') %}{% endif %}
{% endblock class_defs %}
