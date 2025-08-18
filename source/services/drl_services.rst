.. _drl_services:

drl Services
============

.. contents::
   :depth: 1
   :local:


.. _DrlStart:

DrlStart
--------

This is a service to execute a program configured in the DRL language in the robot controller.

**Request:**

.. code-block::

   int8    robot_system       # Robot System Mode: 0 = Real, 1 = Virtual
   string  code               # DRL code to execute

**Response:**

.. code-block::

   bool success

.. note::

   The drl service is only available in real mode.


.. _DrlStop:

DrlStop
-------
This service is used to stop the DRL program (task) currently running on the robot controller. 
Stops differently according to the eStopType received as an argument, and stops the motion of the current section

**Request:**

.. code-block::

   int8    stop_mode          # <STOP_TYPE> stop_mode
                              # STOP_TYPE_QUICK_STO = 0
                              # STOP_TYPE_QUICK     = 1
                              # STOP_TYPE_SLOW      = 2
                              # STOP_TYPE_HOLD = STOP_TYPE_EMERGENCY = 3

**Response:**

.. code-block::

   bool success

.. note::

   The drl service is only available in real mode.

.. _DrlPause:

DrlPause
--------

This is a service to temporarily stop the DRL program (task) currently running on the robot controller.

**Request:**

.. code-block::

   (None)

**Response:**

.. code-block::

   bool success



.. note::

   The drl service is only available in real mode.


.. _DrlResume:

DrlResume
---------

This is a service to resume the currently paused DRL program (task) from the robot controller.

**Request:**

.. code-block::

   (None)

**Response:**

.. code-block::

   bool success

.. note::

   The drl service is only available in real mode.

.. _GetDrlState:

GetDrlState
-----------

Get DRL Program State.

**Request:**

.. code-block::

   (None)

**Response:**

.. code-block::

   int8 drl_state             # <DRL_PROGRAM_STATE>
                              # 0 : DRL_PROGRAM_STATE_PLAY
                              # 1 : DRL_PROGRAM_STATE_STOP
                              # 2 : DRL_PROGRAM_STATE_HOLD
                              # 3 : DRL_PROGRAM_STATE_LAST
   bool success

.. note::

   The drl service is only available in real mode.
