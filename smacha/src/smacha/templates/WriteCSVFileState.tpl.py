{% block meta %}
name: WriteCSVFileState
description:
  SMACH state that writes a comma-separated values (CSV) file to the file system
  based on userdata input.
language: Python
framework: SMACH
type: State
tags: [core]
includes:
extends:
- State
variables:
- input_keys:
    description:
        The userdata input keys, e.g. the data_keys to be written to file.
    type: list of str
- - output_keys:
      description:
          The userdata output keys, e.g. needed by the optionally specified
          callbacks.
      type: list of str
- - callbacks:
      description:
        Either callback function names or backtick-wrapped lambda functions for,
        e.g. modifying the data_keys.
      type: dict of str
- - delimiter:
      description:
        The symbol to be used to separate data columns in the CSV file.
        Defaults to `,`.
      type: list of str
input_keys:
- file:
    description:
      The CSV file path.
    type: str
- data_keys:
    description:
      The names of the input_keys to be written to file.
    type: list of str
output_keys: []
outcomes:
- succeeded
- aborted
{% endblock meta %}

{% from "Utils.tpl.py" import import_module, from_import, render_init_callbacks, render_execute_callbacks, render_callbacks, render_input_keys, render_output_keys, render_transitions, render_remapping %}

{% extends "State.tpl.py" %}

{% block imports %}
{{ super() }}
{{ import_module(defined_headers, 'csv') }}
{% endblock imports %}

{% block class_defs %}
{{ super() }}
{% if 'class_WriteCSVFileState' not in defined_headers %}
class WriteCSVFileState(smach.State):
    """
    This state formats input keys specified by the 'data_keys' input key and writes
    them to a CSV file specified by the 'file' input key.
    """
    def __init__(self, delimiter=',', input_keys = ['file', 'data_keys'], output_keys = [], callbacks=None):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'], input_keys=input_keys, output_keys=output_keys)

        # Save delimiter
        self._delimiter = delimiter

        # Default to ignoring header slots in ROS message types
        # NOTE: This refers to the ROS msg header, NOT the CSV header.
        self._ignore = ['header']

        {{ render_init_callbacks() }}

    def _parse_data_key(self, header, data, data_key, name='', ignore=[]):
        """
        Recursively parse a data key (e.g. a ROS msg type) and return a list of CSV header names and CSV data entries.
        """
        try:
            for slot in data_key.__slots__:
                if slot in ignore:
                    continue
                try:
                    slot_header, slot_data = self._parse_data_key(header, data, getattr(data_key, slot),
                                                                  name=name + '_' + slot, ignore=ignore)
                except:
                    continue
            return header, data
        except:
            try:
                header.append(name)
                data.append(data_key)
                return header, data
            except:
                raise

    def execute(self, userdata):

        {{ render_execute_callbacks() }}

        # Set up ignore slots, if they have been defined by the user
        if 'ignore' in self._input_keys:
            self._ignore = userdata.ignore

        # Set up header
        header = []

        # Set up data
        data = []

        # Try treating data_keys as a singleton ROS type with an iterable slot  
        try:
            # Set up CSV header
            if 'header' in self._input_keys:
                header = userdata.header
                user_defined_header = True
            else:
                header = []
                user_defined_header = False

            # Ensure data_keys contains a single entry
            assert len(userdata.data_keys == 1)
            data_key = userdata.data_keys[0]

            # Look for an iterable slot in the data key
            for slot in data_key.__slots__:
                # Ignore any slots that should be ignored
                if slot in self._ignore:
                    pass
                else:
                    try:
                        for slot_iter_data_key in getattr(data_key, slot):
                            try:
                                if user_defined_header:
                                    _, slot_iter_data_key_data = (
                                            self._parse_data_key([], [], slot_iter_data_key,
                                                                 name=slot, ignore=self._ignore))
                                else:
                                    header, slot_iter_data_key_data = (
                                            self._parse_data_key([], [], slot_iter_data_key,
                                                                 name=slot, ignore=self._ignore))
                                data.append(slot_iter_data_key_data)
                            except:
                                break
                        if data:
                            break
                    except:
                        pass
            assert data
        except:
            # Try iterating through data_keys, using the slots from each item
            # as data entries.
            try:
                # Set up CSV header
                if 'header' in self._input_keys:
                    header = userdata.header
                    user_defined_header = True
                else:
                    header = []
                    user_defined_header = False

                for data_key in userdata.data_keys:
                    # If there is a user-defined header, just parse the data for the data_key entry.
                    if user_defined_header:
                        _, data_key_data = self._parse_data_key([], [], getattr(userdata, data_key),
                                                                name=data_key, ignore=self._ignore)
                    # If not, parse both the data and the header for the data_key entry.
                    else:
                        data_key_header, data_key_data = self._parse_data_key([], [], getattr(userdata, data_key),
                                                                              name=data_key, ignore=self._ignore)
                        header = header + data_key_header
                    data = data + data_key_data
                assert data
                # Ensure data is a list of lists, even if it contains just one list,
                # for the benefit of csv.writer.writerows()
                data = [data]
            except Exception as e:
                smach.logwarn('Failed to parse data keys for CSV file writing: {}'.format(repr(e)))
                return 'aborted'

        # Write to CSV file
        try:
            with open(userdata.file, 'w') as fp:
                writer = csv.writer(fp, delimiter=self._delimiter)
                if header:
                    writer.writerow(header)
                writer.writerows(data)
        except Exception as e:
            smach.logwarn('Failed to write to CSV file \'{}\': {}'.format(userdata.file, repr(e)))
            return 'aborted'

        # Write the header as output key if required.
        if 'header' in self._output_keys:
            userdata.header = header

        return 'succeeded'
{% do defined_headers.append('class_WriteCSVFileState') %}{% endif %}
{% endblock class_defs %}

{% block body %}
smach.{{ parent_type }}.add('{{ name }}',
{{ '' | indent(23, true) }}WriteCSVFileState({% if delimiter is defined %}delimiter='{{ delimiter }}'{% endif %}{% if input_keys is defined %}{% if delimiter is defined %}, {% endif %}{{ render_input_keys(input_keys, indent=0) }}{% endif %}{% if output_keys is defined %}{% if input_keys is defined or delimiter is defined %}, {% endif %}{{ render_output_keys(output_keys, indent=0) }}{% endif %}{% if callbacks is defined %}, {{ render_callbacks(name, uuid, callbacks, indent=0) }}{% endif %}){% if transitions is defined %},
{{ render_transitions(transitions) }}{% endif %}{% if remapping is defined %},
{{ render_remapping(remapping) }}{% endif %})
{% endblock body %}
