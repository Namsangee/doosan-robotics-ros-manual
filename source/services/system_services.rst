.. _system_services:

system Services
===============

.. contents::
   :depth: 1
   :local:

.. _GetRobotMode:

GetRobotMode
------------
It is a service input for checking the current operation mode of the robot controller.

The auto mode is a mode for automatically performing a series of operations (programs), and the manual mode is for performing a single operation such as jogging.

**Request:**

.. code-block::

   (None)

**Response:**

.. code-block::

   int8 robot_mode
   bool        success

.. _enum_robot_mode:

**enum.ROBOT_MODE**

.. list-table::
   :header-rows: 1
   :widths: 10 40 50

   * - Num
     - Name
     - Description
   * - 0
     - ROBOT_MODE_MANUAL
     - Manual mode
   * - 1
     - ROBOT_MODE_AUTONOMOUS
     - Auto mode
   * - 2
     - ROBOT_MODE_MEASURE
     - Measure mode (Not currently supported)




.. _SetRobotMode:

SetRobotMode
------------

This service is for setting the current operation mode of the robot controller.

**Request:**

.. code-block::

   int8 robot_mode # <Robot_Mode>

**Response:**

.. code-block::

   bool success

**enum.ROBOT_MODE**

.. list-table::
   :header-rows: 1
   :widths: 10 40 50

   * - Num
     - Name
     - Description
   * - 0
     - ROBOT_MODE_MANUAL
     - Manual mode
   * - 1
     - ROBOT_MODE_AUTONOMOUS
     - Auto mode
   * - 2
     - ROBOT_MODE_MEASURE
     - Measure mode (Not currently supported)

.. _GetRobotSystem:

GetRobotSystem
--------------
It is a service input for confirming the current operation mode (virtual robot, actual robot) of the robot controller.

**Request:**

.. code-block::

   (None)

**Response:**

.. code-block::

   int8 robot_system   # 0 : ROBOT_SYSTEM_REAL
                       # 1 : ROBOT_SYSTEM_VIRTUAL
   bool        success


.. _SetRobotSystem:

SetRobotSystem
--------------
This is a service for setting up the current robot system of the robot controller.


**Request:**

.. code-block::

   int8 robot_system   # 0 : ROBOT_SYSTEM_REAL
                       # 1 : ROBOT_SYSTEM_VIRTUAL

**Response:**

.. code-block::

   bool success


.. _GetRobotSpeedMode:

GetRobotSpeedMode
-----------------
This service is used to check the current speed mode (normal mode, deceleration mode) from the robot controller.


**Request:**

.. code-block::

   (None)

**Response:**

.. code-block::

   int8 speed_mode      # 0 : SPEED_NORMAL_MODE
                        # 1 : SPEED_REDUCED_MODE
   bool        success   


.. _SetRobotSpeedMode:

SetRobotSpeedMode
-----------------
This service is used to set and change the currently operating speed mode of the robot controller.

**Request:**

.. code-block::

   int8 speed_mode      # 0 : SPEED_NORMAL_MODE, 
                        # 1 : SPEED_REDUCED_MODE

**Response:**

.. code-block::

   bool success    
                   


.. _SetSafeStopResetType:

SetSafeStopResetType
--------------------
This service is used to define a series of actions to be executed automatically after the state transition using the SetRobotMode service when the operation status information of the robot controller is SAFE_STOP.

If the robot operation mode is automatic, you can define and set whether to re-execute the program. In manual mode, this setting is ignored.

**Request:**

.. code-block::

   int8 reset_type      # 0: SAFE_STOP_RESET_TYPE_DEFAULT = SAFE_STOP_RESET_TYPE_PROGRAM_STOP
                        # 1: SAFE_STOP_RESET_TYPE_PROGRAM_RESUME 

**Response:**

.. code-block::

   bool success


**enum.SAFE_STOP_RESET_TYPE**

.. list-table::
   :header-rows: 1
   :widths: 10 45 45

   * - Num
     - Name
     - Description
   * - 0
     - SAFE_STOP_RESET_TYPE_DEFAULT
     - Simple state release (manual mode)
   * - 0
     - SAFE_STOP_RESET_TYPE_PROGRAM_STOP
     - Stop program (auto mode)
   * - 1
     - SAFE_STOP_RESET_TYPE_PROGRAM_RESUME
     - Restart the program (automatic mode)

.. _SetSafetyMode:

SetSafetyMode
-------------
Set the controller **safety mode** and associated **safety event**.

**Request:**

.. code-block::

   int8 safety_mode
   int8 safety_event

**Response:**

.. code-block::

   bool success

.. _enum_safety_mode:

**enum.SAFETY_MODE**

.. list-table::
   :header-rows: 1
   :widths: 10 40 50

   * - Num
     - Name
     - Description
   * - 0
     - SAFETY_MODE_MANUAL
     - Manual safety mode
   * - 1
     - SAFETY_MODE_AUTONOMOUS
     - Autonomous safety mode
   * - 2
     - SAFETY_MODE_RECOVERY
     - Recovery safety mode
   * - 3
     - SAFETY_MODE_BACKDRIVE
     - Backdrive safety mode
   * - 4
     - SAFETY_MODE_MEASURE
     - Measure safety mode
   * - 5
     - SAFETY_MODE_INITIALIZE
     - Initialize safety mode


.. _enum_safety_mode_event:

**enum.SAFETY_MODE_EVENT**


.. list-table::
   :header-rows: 1
   :widths: 10 40 50

   * - Num
     - Name
     - Description
   * - 0
     - SAFETY_MODE_EVENT_ENTER
     - Enter the safety mode
   * - 1
     - SAFETY_MODE_EVENT_MOVE
     - Move within the safety mode
   * - 2
     - SAFETY_MODE_EVENT_STOP
     - Stop within the safety mode
   * - 3
     - SAFETY_MODE_EVENT_LAST
     - Last event (sentinel)


.. _GetCurrentPose:


GetCurrentPose
--------------
This service is used to check the current position information of each axis of the robot 
according to the coordinate system (joint space or task space) in the robot controller.

**Request:**

.. code-block::

   int8 space_type      # 0: ROBOT_SPACE_JOINT
                        # 1: ROBOT_SPACE_TASK

**Response:**

.. code-block::

   float64[6] pos
   bool       success

.. _GetLastAlarm:

GetLastAlarm
------------
This service is used to check the most recent log and alarm codes generated by the robot controller.

**Request:**

.. code-block::

   (None)

**Response:**

.. code-block::

   LogAlarm    log_alarm
   bool        success

.. _logalarm_msg:

**LogAlarm.msg**

.. list-table::
   :header-rows: 1
   :widths: 22 18 18 42

   * - Parameter Name
     - Data Type
     - Default Value
     - Description
   * - level
     - int32
     - -
     - refer to :ref:`enum.LOG_LEVEL <enum_log_level>`
   * - group
     - int32
     - -
     - refer to :ref:`enum.LOG_GROUP <enum_log_group>`
   * - index
     - int32
     - -
     - error code
   * - param
     - string[]
     - -
     - param(s)

.. _enum_log_level:

**enum.LOG_LEVEL**

.. list-table::
   :header-rows: 1
   :widths: 10 35 55

   * - Num
     - Name
     - Description
   * - 0
     - LOG_LEVEL_RESERVED
     - reserved
   * - 1
     - LOG_LEVEL_SYSINFO
     - Informational messages about basic functions and operational errors
   * - 2
     - LOG_LEVEL_SYSWARN
     - Robot is stopped due to a basic function or operation error
   * - 3
     - LOG_LEVEL_SYSERROR
     - Robot is stopped due to a safety issue or device error

.. _enum_log_group:

**enum.LOG_GROUP**

.. list-table::
   :header-rows: 1
   :widths: 10 40 50

   * - Num
     - Name
     - Description
   * - 0
     - LOG_GROUP_RESERVED
     - reserved
   * - 1
     - LOG_GROUP_SYSTEMFMK
     - framework
   * - 2
     - eLOG_GROUP_MOTIONLIB
     - Motion algorithm
   * - 3
     - LOG_GROUP_SMARTTP
     - TP program (GUI)
   * - 4
     - LOG_GROUP_INVERTER
     - Robot Inverter Board
   * - 5
     - LOG_GROUP_SAFETYCONTROLLER
     - Safety Controller



.. _ChangeCollisionSensitivity:

ChangeCollisionSensitivity
--------------------------
This is a function to configure the collision sensitivity in the robot controller.


**Request:**

.. code-block::

   int8 sensitivity   # Colision Sensitivity(0~100)

**Response:**

.. code-block::

   bool success



.. _GetRobotState:

GetRobotState
-------------
This is a service for checking information on the current operation mode of the robot controller, 
and the user should transfer the operation state depending on the state using the SetRobotControl service for safety.

**Request:**

.. code-block::

   (None)

**Response:**

.. code-block::

   int8 robot_state     # 0 : STATE_INITIALIZING
                        # 1 : STATE_STANDBY
                        # 2 : STATE_MOVING
                        # 3 : STATE_SAFE_OFF
                        # 4 : STATE_TEACHING
                        # 5 : STATE_SAFE_STOP
                        # 6 : STATE_EMERGENCY_STOP
                        # 7 : STATE_HOMMING
                        # 8 : STATE_RECOVERY
                        # 9 : eSTATE_SAFE_STOP2
                        # 10: STATE_SAFE_OFF2
                        # 11: STATE_RESERVED1
                        # 12: STATE_RESERVED2
                        # 13: STATE_RESERVED3
                        # 14: STATE_RESERVED4
                        # 15: STATE_NOT_READY
   bool        success                    


**enum.ROBOT_STATE**

.. list-table::
   :header-rows: 1
   :widths: 8 32 60

   * - Num
     - Name
     - Description
   * - 0
     - STATE_INITIALIZING
     - Initialization state entered automatically by the T/P application; used to initialize various parameters.
   * - 1
     - STATE_STANDBY
     - Operable base state; command standby.
   * - 2
     - STATE_MOVING
     - Command-execution state while the robot is moving; when motion finishes, it returns to command standby automatically.
   * - 3
     - STATE_SAFE_OFF
     - Robot pause mode caused by functional/operational error; servo-off state (motor and brake power cut after control pause).
   * - 4
     - STATE_TEACHING
     - Direct teaching state.
   * - 5
     - STATE_SAFE_STOP
     - Safety stop state caused by functional/operational error; a control pause is executed (temporary program pause in auto mode).
   * - 6
     - STATE_EMERGENCY_STOP
     - Emergency stop state.
   * - 7
     - STATE_HOMING
     - Homing mode state (hardware-based home search of the robot).
   * - 8
     - STATE_RECOVERY
     - Recovery mode to move the robot back into the operation range after errors such as leaving the operation range.
   * - 9
     - STATE_SAFE_STOP2
     - State indicating recovery is required due to leaving the operation range; equivalent to ``STATE_SAFE_STOP``.
   * - 10
     - STATE_SAFE_OFF2
     - State indicating recovery is required due to leaving the operation range; equivalent to ``STATE_SAFE_OFF``.
   * - 11
     - STATE_RESERVED1
     - Reserved.
   * - 12
     - STATE_RESERVED2
     - Reserved.
   * - 13
     - STATE_RESERVED3
     - Reserved.
   * - 14
     - STATE_RESERVED4
     - Reserved.
   * - 15
     - STATE_NOT_READY
     - Post boot-up pre-initialization state of the robot controller; converted to the initialization state by the T/P application.

.. _ServoOff:

ServoOff
--------
This is a service to set the motor and brake power (robot power off) in the robot controller.

**Request:**

.. code-block::

   int8 STOP_TYPE_QUICK_STO = 0
   int8 STOP_TYPE_QUICK = 1
   int8 STOP_TYPE_SLOW = 2
   int8 STOP_TYPE_HOLD = 3
   int8 STOP_TYPE_EMERGENCY = 3

   int8 stop_type     

**Response:**

.. code-block::

   bool success

**enum.STOP_TYPE**

.. list-table::
   :header-rows: 1
   :widths: 10 40 50

   * - Num
     - Name
     - Description
   * - 0
     - STOP_TYPE_QUICK_STO
     - Internal reservation used
   * - 1
     - STOP_TYPE_QUICK
     - Quick Stop (maintenance of motion trajectory)
   * - 2
     - STOP_TYPE_SLOW
     - Slow Stop (maintenance of motion trajectory)
   * - 3
     - STOP_TYPE_HOLD
     - Emergency Stop
   * - -
     - STOP_TYPE_EMERGENCY
     - Emergency Stop

.. _SetRobotControl:

SetRobotControl
---------------
This is a service that the user can set and convert the current operation state in the robot controller.

**Request:**

.. code-block::

   int8 robot_control 

**Response:**

.. code-block::

   bool success

**enum.ROBOT_CONTROL**

.. list-table::
   :header-rows: 1
   :widths: 8 32 60

   * - Num
     - Name
     - Description
   * - 0
     - CONTROL_INIT_CONFIG
     - Converts from ``STATE_NOT_READY`` to ``STATE_INITIALIZING``
       ; only the T/P application executes this function.
   * - 1
     - CONTROL_ENABLE_OPERATION
     - Converts from ``STATE_INITIALIZING`` to ``STATE_STANDBY``
       ; only the T/P application executes this function.
   * - 2
     - CONTROL_RESET_SAFE_STOP
     - Converts from ``STATE_SAFE_STOP`` to ``STATE_STANDBY``
       In automatic mode, a program restart can be configured.
   * - 3
     - CONTROL_RESET_SAFE_OFF
     - Converts from ``STATE_SAFE_OFF`` to ``STATE_STANDBY``.
   * - 4
     - CONTROL_RECOVERY_SAFE_STOP
     - S/W-based recovery: converts from ``STATE_SAFE_STOP2`` to ``STATE_RECOVERY``.
   * - 5
     - CONTROL_RECOVERY_SAFE_OFF
     - S/W-based recovery: converts from ``STATE_SAFE_OFF2`` to ``STATE_RECOVERY``.
   * - 6
     - CONTROL_RECOVERY_BACKDRIVE
     - H/W-based recovery: converts from ``STATE_SAFE_OFF2`` to ``STATE_RECOVERY`` 
       It cannot transition directly to ``STATE_STANDBY``; reboot the robot controller power.
   * - 7
     - CONTROL_RESET_RECOVERY
     - Converts from ``STATE_RECOVERY`` to ``STATE_STANDBY``.
