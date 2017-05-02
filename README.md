# SMACHA

SMACHA is a [Jinja2](http://jinja.pocoo.org/docs/2.9/) and [YAML](http://yaml.org/)-based
code templating, generation and scripting engine for [SMACH](http://wiki.ros.org/smach).

[SMACH](http://wiki.ros.org/smach) is an exceptionally useful and comprehensive task-level architecture
for state machine construction in [ROS](http://wiki.ros.org/)-based robot control systems.
However, while it provides much in terms of power and flexibility, its overall task-level simplicity
can often be obfuscated at the script-level by boilerplate code, intricate structure and lack
of code reuse between state machine prototypes.

SMACHA (pronounced "smasha") aims at distilling the task-level simplicity of SMACH into compact YAML-based scripts
in the foreground, while retaining all of its power and flexibility in Jinja2-based
templates and a custom code generation engine in the background.

## SMACHAML Scripting

SMACHAML ("SMACH As Markup Language", pronounced "smashamel") scripts are YAML scripts for describing how SMACHA should
generate SMACH code. Here is the ["Nesting State Machines" example](http://wiki.ros.org/smach/Tutorials/Nesting%20State%20Machines)
from the [SMACH Tutorials](http://wiki.ros.org/smach/Tutorials) described in SMACHAML:
```yaml
--- # Nesting State Machines Tutorial SMACHAML script.
name: sm_top
template: Base
node_name: smach_example_state_machine
outcomes: [outcome5]
states:
  - BAS:
      template: Bas
      transitions: {outcome3: SUB}
  - SUB:
      template: StateMachine
      outcomes: [outcome4]
      transitions: {outcome4: outcome5}
      states:
        - FOO:
            template: Foo
            transitions: {outcome1: BAR, outcome2: outcome4}
        - BAR:
            template: Bar
            transitions: {outcome1: FOO}
```
This demonstrates a reduction from 80 lines of raw SMACH code to 20 lines of SMACHAML. Nice.

### Base

The base of the script specifies a name for the overall state machine,
the name of its base template, a name for its associated ROS node,
a list of its possible outcomes, and list of its constituent states.

### Templates

Each state, including the base, must specify a template from which its respective
code should be generated.  More on that in the next section.

### States

States may be specified as lists, specifying their transition order,
and may also be [nested](http://wiki.ros.org/smach/Tutorials/Nesting%20State%20Machines)
as described in the SMACH tutorials using appropriate combinations of
template and states specification as seen in the example above.

### Outcomes

Possible state outcomes may be specified as a list in each state.

### Transitions
Possible state transitions may be specified as a hash/dictionary in each state.

## Jinja2 Templating

[Jinja2](http://jinja.pocoo.org/docs/2.9/) templates are used to specify how code should be
generated from SMACHAML scripts.
The `Base` template from the above example is specified in a `Base.jinja` file and looks like this:

```python
#!/usr/bin/env python

import roslib; roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros

{{ header }}

def main():
    rospy.init_node('{{ node_name }}')
    
    # Create a SMACH state machine
    {{ name | lower() }} = smach.StateMachine(outcomes=[{% for outcome in outcomes %}'{{ outcome }}'{% if not loop.last %}, {% endif %}{% endfor %}])
    
    # Open the container
    with {{ name | lower() }}:
        # Add states to the container
        {{ body | indent(8) }}
    
    # Execute SMACH plan
    outcome = {{ name | lower() }}.execute()
    
    {{ footer | indent(4) }}

if __name__ == '__main__':
    main()
```

The `Foo` template is specified in a `Foo.jinja` file and looks like this:
```python
{% block base_header %}
# define state Foo
class Foo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state {{ name }}')
        if self.counter < 3:
            self.counter += 1
            return 'outcome1'
        else:
            return 'outcome2'
{% endblock base_header %}

{% block body %}
smach.StateMachine.add('{{ name }}', Foo(), 
                       transitions={{ '{' }}{% for outcome, transition in transitions.iteritems() %}'{{ outcome }}':'{{ transition }}'{% if not loop.last %},
                                    {% endif %}{% endfor %}{{ '}' }})
{% endblock body %}

{% block base_footer %}
{% endblock base_footer %}
```
### Variables and Code Blocks

The `base_header` block in a state template is rendered into the `header` variable
in the base template.

The `body` block in a state template is rendered into the `body` variable of either its
parent state or the base template.

The `base_footer` block in a state template is rendered into the `footer` variable
in the base template.

### Core Templates

Default core templates are provided for many of the SMACH containers, as well as other
SMACH constructs like action states.

These core templates may be overridden or augmented by custom user-specified templates.
See the [Usage](#Usage) section below for an example.

TODO: Finish adding core templates and describing them here.

## Code Generation Engine

The SMACHA code generator is a Python API that recursively processes SMACHAML scripts,
manages Python code templates, and renders them to Python SMACH code.

The API can either be imported and used in Python projects or invoked as a command-line
script via `rosrun`.

A ROS wrapper may eventually be written to allow the generator to be launched as a ROS
node that exposes services and generates SMACH code over ROS topics, but this has not yet
been implemented.

## Installation

Simply clone into the `src` directory of your catkin workspace and run `catkin_make` from the
root of the workspace.

## Usage
In the simplest case, using default core templates, SMACHA can be invoked on a `my_script.yml`
SMACHAML file as follows:
```
rosrun smacha smacha my_script.yml
```
where the generated code will be output to a file called `smacha_output.py`

Example usage for the "Nesting State Machines" tutorial:
```
roscd smacha/test
rosrun smacha smacha smachaml/executive_smach_tutorials/state_machine_nesting2.yml -t templates/executive_smach_tutorials/state_machine2 templates/executive_smach_tutorials/state_machine_nesting2 -o state_machine_nesting2.py -v
```
Here, the `-t` argument specifies custom template directories for this particular tutorial.
In this case, two directories are specified such that templates from the preceding
["Simple State Machine" tutorial](http://wiki.ros.org/smach/Tutorials/Simple%20State%20Machine)
may be reused.

The `-o` argument specifies a custom name for the generated output file.

The `-v` argument tells SMACHA to print verbose processing output to the terminal.

Further arguments and options may be explored by running `rosrun smacha smacha -h` or `rosrun smacha smacha --help`.

Tests may be performed by running:
```
roscd smacha/teset
nosetests executive_smach_tutorials.py
```

## Contributors
SMACHA is developed and maintained by [Barry Ridge](https://barog.net/).

## Acknowledgements
SMACHA was developed for the [EU H2020 ReconCell Project](http://www.reconcell.eu/).
With thanks to [Minija Tamošiūnaitė](http://www.dpi.physik.uni-goettingen.de/cns/index.php?mact=Profilliste,cntnt01,default,0&cntnt01what=Mitarbeiter&cntnt01alias=Tamosiunaite&cntnt01returnid=65) for design contributions.
