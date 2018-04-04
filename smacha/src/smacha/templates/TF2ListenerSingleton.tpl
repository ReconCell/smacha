# 
# Based on the original code described here:
# http://wiki.ros.org/executive_smach/AdditionalStateTypes
# https://github.com/felix-kolbe/uashh-rvl-ros-pkg/blob/master/uashh_smach/src/uashh_smach/util.py
#
{% from "Utils.tpl" import import_module %}

{% block imports %}
{{ import_module(defined_headers, 'tf2_ros') }}
{% endblock imports %}

{% block class_defs %}
{% if 'class_TF2ListenerSingleton' not in defined_headers %}
class TF2ListenerSingleton(object):
    """To avoid running multiple transform listeners, this singleton class
    provides one tf2 buffer and listener that are initialised and retrieved via
    class methods init() and get().
    """
    _buffer = None
    _listener = None

    @classmethod
    def init(cls):
        """Ignores multiple calls."""
        if cls._buffer is None or cls._listener is None:
            cls._buffer = tf2_ros.Buffer()
            cls._listener = tf2_ros.TransformListener(cls._buffer)

    @classmethod
    def get(cls):
        """Does initialise if needed, too."""
        cls.init()
        return cls._buffer, cls._listener
{% do defined_headers.append('class_TF2ListenerSingleton') %}{% endif %}
{% endblock class_defs %}
