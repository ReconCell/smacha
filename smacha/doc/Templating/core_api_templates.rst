Core Base Templates
===================

The Base Template
-----------------

:doc:`../API/Templates/Base.tpl.py` is used for specifying the bare bones of a Python SMACH state machine script and is specified as follows:

.. program-output:: ../scripts/help Base -n

Core Container Templates
========================

The StateMachine Template
-------------------------

:doc:`../API/Templates/StateMachine.tpl.py` is used for inserting a `StateMachine container <http://wiki.ros.org/smach/Tutorials/StateMachine%20container>`__ and is specified as follows: 

.. program-output:: ../scripts/help StateMachine -n

The Concurrence Template
------------------------

:doc:`../API/Templates/Concurrence.tpl.py` is used for inserting a `Concurrence container <http://wiki.ros.org/smach/Tutorials/Concurrence%20container>`__ and is specified as follows:

.. program-output:: ../scripts/help Concurrence -n

Core State Templates
====================

The CallbacksState Template
---------------------------

:doc:`../API/Templates/CallbacksState.tpl.py` is used for creating lambda function callbacks and is specified as follows:

.. program-output:: ../scripts/help CallbacksState -n

The PrintUserdataState Template
-------------------------------

:doc:`../API/Templates/PrintUserdataState.tpl.py` is used for printing userdata entries to standard output and is specified as follows:

.. program-output:: ../scripts/help PrintUserdataState -n

The RandomOutcomeState Template
-------------------------------

:doc:`../API/Templates/RandomOutcomeState.tpl.py` is used for selecting a random outcome from a specified list of outcomes and is specified as follows:

.. program-output:: ../scripts/help RandomOutcomeState -n

The ServiceState Template
-------------------------

:doc:`../API/Templates/ServiceState.tpl.py` is used for inserting a `ServiceState <http://wiki.ros.org/smach/Tutorials/ServiceState>`__ and is specified as follows:

.. program-output:: ../scripts/help ServiceState -n

The SimpleActionState Template
------------------------------

:doc:`../API/Templates/SimpleActionState.tpl.py` is used for inserting a `SimpleActionState <http://wiki.ros.org/smach/Tutorials/SimpleActionState>`__ and is specified as follows:

.. program-output:: ../scripts/help SimpleActionState -n

The TF2ListenerState Template
-----------------------------

:doc:`../API/Templates/TF2ListenerState.tpl.py` is used for reading TF2 transforms and is specified as follows:

.. program-output:: ../scripts/help TF2ListenerState -n


Other Core Templates
====================

The State Template
------------------

:doc:`../API/Templates/State.tpl.py` contains code common to all state templates and is specified as follows:

.. program-output:: ../scripts/help State -n

The Utils Template
------------------

:doc:`../API/Templates/Utils.tpl.py` contains template macros and other utilities and is specified as follows:

.. program-output:: ../scripts/help Utils -n

The TF2ListenerSingleton Template
---------------------------------

:doc:`../API/Templates/TF2ListenerSingleton.tpl.py` provides a helper class for :doc:`../API/Templates/TF2ListenerState.tpl.py` and is specified as follows:

.. program-output:: ../scripts/help TF2ListenerSingleton -n


