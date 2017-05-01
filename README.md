# SMACHA

SMACHA is a [Jinja2](http://jinja.pocoo.org/docs/2.9/) and [YAML](http://yaml.org/)-based
code templating, generation and scripting engine for [SMACH](http://wiki.ros.org/smach).

## Why?

[SMACH](http://wiki.ros.org/smach) is an exceptionally useful and comprehensive task-level architecture
for state machine construction in [ROS](http://wiki.ros.org/)-based robot control systems.
However, while it provides much in terms of power and flexibility, its overall task-level simplicity
can often be obfuscated at the script-level by boilerplate code, intricate structure and lack
of code reuse between state machine prototypes.

SMACHA aims at distilling the task-level simplicity of SMACH into compact YAML-based scripts
in the foreground, while retaining all of its power and flexibility in Jinja2-based
templates and a custom code generation engine in the background.

## How?

### SMACHAML Scripting

SMACHAML ("SMACH As Markup Language") scripts are YAML scripts for describing how SMACHA should
generate SMACH code.

### Jinja2 Templates

### Code Generation

