{% from "Utils.tpl" import render_userdata %}

{% set local_vars = [] %}

{% block header %}
{% if userdata is defined %}{{ render_userdata(parent_sm_name, userdata) }}{% endif %}
{% endblock header %}
