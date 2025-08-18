.. _realtime_services:

Real-time Services
==================

.. contents::
   :depth: 1
   :local:

.. _ConnectRtControl:

ConnectRtControl
----------------
Connect to the robot controller via Real-time External Control.

**Request:**

.. code-block::

   string  ip_address
   uint32  port

**Response:**

.. code-block::

   bool    success

.. note::

   - Real-time external control uses **UDP/IP** communication.
   - This channel is **independent** of the standard **TCP/IP API** and does not rely on API control authority.
   - For the integrated controller (v3), versions **v3.2.2 and earlier** can be reached at **192.168.137.50**.

.. caution::

   - Currently, only **one-to-one** communication is supported (one external controller ↔ one robot controller).

.. _DisconnectRtControl:

DisconnectRtControl
-------------------
Disconnect from Real-time External Control.

**Request:**

.. code-block::

   (None)

**Response:**

.. code-block::

   bool    success

.. note::

   All real-time control settings are **reset** on disconnect.

.. _GetRtControlInputDataList:

GetRtControlInputDataList
-------------------------
Return the list of **input data** supported by a specific version.

**Request:**

.. code-block::

   string  version

**Response:**

.. code-block::

   bool    success
   string  data


**Input Data List**

.. list-table::
   :header-rows: 1
   :widths: 28 12 40 20

   * - Parameter Name
     - Data Type
     - Description
     - First Updated Controller Version
   * - external_force_torque
     - float[6]
     - external force/torque sensor
     - v1.0
   * - external_digital_input
     - uint16
     - external digital input (16 channel)
     - v1.0
   * - external_digital_output
     - uint16
     - external digital output (16 channel)
     - v1.0
   * - external_analog_input
     - float[6]
     - external analog input (6 channel)
     - v1.0
   * - external_analog_output
     - float[6]
     - external analog output (6 channel)
     - v1.0

.. _GetRtControlInputVersionList:

GetRtControlInputVersionList
----------------------------
Return the list of Real-time External Control **versions** for input data.

**Request:**

.. code-block::

   (None)

**Response:**

.. code-block::

   bool    success
   string  version

.. _GetRtControlOutputDataList:

GetRtControlOutputDataList
--------------------------
Return the list of **output data** supported by a specific version.

**Request:**

.. code-block::

   string  version

**Response:**

.. code-block::

   bool    success
   string  data

.. _output_data_list_runtime:

**Output Data List**

.. list-table::
   :header-rows: 1
   :widths: 26 16 46 12

   * - Parameter Name
     - Data Type
     - Description
     - First Updated Controller Version
   * - time_stamp
     - double
     - timestamp at the moment of data acquisition [s]
     - v1.0
   * - actual_joint_position
     - float64[6]
     - actual joint position from incremental encoder at motor side (used for control) [deg]
     - v1.0
   * - actual_joint_position_abs
     - float64[6]
     - actual joint position from absolute encoder at link side (used for exact link position) [deg]
     - v1.0
   * - actual_joint_velocity
     - float64[6]
     - actual joint velocity from incremental encoder at motor side [deg/s]
     - v1.0
   * - actual_joint_velocity_abs
     - float64[6]
     - actual joint velocity from absolute encoder at link side [deg/s]
     - v1.0
   * - actual_tcp_position
     - float64[6]
     - actual TCP position w.r.t. base coordinates: (x, y, z, a, b, c), where (a, b, c) follows Euler ZYZ notation [mm, deg]
     - v1.0
   * - actual_tcp_velocity
     - float64[6]
     - actual TCP velocity w.r.t. base coordinates [mm, deg/s]
     - v1.0
   * - actual_flange_position
     - float64[6]
     - actual flange position w.r.t. base coordinates: (x, y, z, a, b, c), where (a, b, c) follows Euler ZYZ notation [mm, deg]
     - v1.0
   * - actual_flange_velocity
     - float64[6]
     - flange velocity w.r.t. base coordinates [mm, deg/s]
     - v1.0
   * - actual_motor_torque
     - float64[6]
     - actual motor torque applying gear ratio = gear_ratio × current2torque_constant × motor current [Nm]
     - v1.0
   * - actual_joint_torque
     - float64[6]
     - estimated joint torque by robot controller [Nm]
     - v1.0
   * - raw_joint_torque
     - float64[6]
     - calibrated joint torque sensor data
     - v1.0
   * - raw_force_torque
     - float64[6]
     - calibrated force/torque sensor data w.r.t. flange coordinates [N, Nm]
     - v1.0
   * - external_joint_torque
     - float64[6]
     - estimated joint torque [Nm]
     - v1.0
   * - external_tcp_force
     - float64[6]
     - estimated TCP force w.r.t. base coordinates [N, Nm]
     - v1.0
   * - target_joint_position
     - float64[6]
     - target joint position [deg]
     - v1.0
   * - target_joint_velocity
     - float64[6]
     - target joint velocity [deg/s]
     - v1.0
   * - target_joint_acceleration
     - float64[6]
     - target joint acceleration [deg/s^2]
     - v1.0
   * - target_motor_torque
     - float64[6]
     - target motor torque [Nm]
     - v1.0
   * - target_tcp_position
     - float64[6]
     - target TCP position w.r.t. base coordinates: (x, y, z, a, b, c), where (a, b, c) follows Euler ZYZ notation [mm, deg]
     - v1.0
   * - target_tcp_velocity
     - float64[6]
     - target TCP velocity w.r.t. base coordinates [mm, deg/s]
     - v1.0
   * - jacobian_matrix
     - float64[6][6]
     - Jacobian matrix :math:`J(q)` w.r.t. base coordinates
     - v1.0
   * - gravity_torque
     - float64[6]
     - gravity torque :math:`g(q)` [Nm]
     - v1.0
   * - coriolis_matrix
     - float64[6][6]
     - Coriolis matrix :math:`C(q)` [Nm·s]
     - v1.0
   * - mass_matrix
     - float64[6][6]
     - Mass matrix :math:`M(q)+B` [Nm·s^2]
     - v1.0
   * - solution_space
     - uint8
     - robot configuration
     - v1.0
   * - singularity
     - float64
     - minimum singular value
     - v1.0
   * - operation_speed_rate
     - float64
     - current operation speed rate (1–100 %)
     - v1.0
   * - joint_temperature
     - float64[6]
     - joint temperature (°C)
     - v1.0
   * - controller_digital_input
     - uint16
     - controller digital input (16 channel)
     - v1.0
   * - controller_digital_output
     - uint16
     - controller digital output (16 channel)
     - v1.0
   * - controller_analog_input_type
     - uint8
     - controller analog input type (2 channel)
     - v1.0
   * - controller_analog_input
     - float64[2]
     - controller analog input (2 channel)
     - v1.0
   * - controller_analog_output_type
     - uint8
     - controller analog output type (2 channel)
     - v1.0
   * - controller_analog_output
     - float64[2]
     - controller analog output (2 channel)
     - v1.0
   * - flange_digital_input
     - uint8
     - flange digital input (A-Series: 2 channel, M/H-Series: 6 channel)
     - v1.0
   * - flange_digital_output
     - uint8
     - flange digital output (A-Series: 2 channel, M/H-Series: 6 channel)
     - v1.0
   * - flange_analog_input
     - float64[4]
     - flange analog input (A-Series: 2 channel, M/H-Series: 4 channel)
     - v1.0
   * - external_encoder_strobe_count
     - uint8[2]
     - strobe count (increments by 1 when detecting setting edge)
     - v1.0
   * - external_encoder_count
     - uint32[2]
     - external encoder count
     - v1.0
   * - goal_joint_position
     - float64[6]
     - final goal joint position (reserved)
     - v1.0
   * - goal_tcp_position
     - float64[6]
     - final goal TCP position (reserved)
     - v1.0
   * - robot_mode
     - uint8
     - ROBOT_MODE_MANUAL(0), ROBOT_MODE_AUTONOMOUS(1), ROBOT_MODE_MEASURE(2)
     - v1.0
   * - robot_state
     - uint8
     - STATE_INITIALIZING(0), STATE_STANDBY(1), STATE_MOVING(2), STATE_SAFE_OFF(3), STATE_TEACHING(4), STATE_SAFE_STOP(5), STATE_EMERGENCY_STOP(6), STATE_HOMING(7), STATE_RECOVERY(8), STATE_SAFE_STOP2(9), STATE_SAFE_OFF2(10)
     - v1.0
   * - control_mode
     - uint16
     - position control mode, torque mode
     - v1.0

.. _GetRtControlOutputVersionList:

GetRtControlOutputVersionList
-----------------------------
Return the list of Real-time External Control **versions** for output data.

**Request:**

.. code-block::

   (None)

**Response:**

.. code-block::

   bool    success
   string  version

**Output Data Version List**

.. list-table::
   :header-rows: 1
   :widths: 40 60

   * - Output Data Version
     - First Updated Controller Version
   * - v1.0
     - V2.9

.. _ReadDataRt:

ReadDataRt
----------
Read the real-time **output** data from the robot controller.

**Request:**

.. code-block::

   (None)

**Response:**

.. code-block::

   RobotStateRt  data

.. _SetAccjRt:

SetAccjRt
---------
Set the **global joint acceleration limit** used by real-time servo motion.

**Request:**

.. code-block::

   float64[6] acc    # [deg/s^2]

**Response:**

.. code-block::

   bool       success

.. note::

   If the measured acceleration during servo motion exceeds the global limit, an **info message** is generated.

.. _SetAccxRt:

SetAccxRt
---------
Set the **global task acceleration limit** used by real-time servo motion.

**Request:**

.. code-block::

   float64  trans     # task linear acceleration limit [mm/s^2]
   float64  rotation  # task rotational acceleration limit [deg/s^2]
                      # if omitted, it is auto-calculated from the linear acceleration limit

**Response:**

.. code-block::

   bool     success

.. note::

   If the measured acceleration during servo motion exceeds the global limit, an **info message** is generated.

.. _SetRtControlInput:

SetRtControlInput
-----------------
Configure the **input data** stream (external controller → robot controller) for Real-time External Control.

**Request:**

.. code-block::

   string  version   # Input Data Version
   float64 period    # communication period [s], range: 0.001 ~ 1.0
   int32   loss      # if input data or servo commands are missed consecutively
                     # more than the configured loss count, the real-time link is disconnected.
                     # set to -1 to disable this check.

**Response:**

.. code-block::

   bool    success

.. note::

   ``period`` currently supports **0.001 to 1.0 s**.

.. _SetRtControlOutput:

SetRtControlOutput
------------------
Configure the **output data** stream (robot controller → external controller) for Real-time External Control.

**Request:**

.. code-block::

   string  version   # Output Data Version
   float64 period    # communication period [s], range: 0.001 ~ 1.0
   int32   loss      # loss count (currently unused)

**Response:**

.. code-block::

   bool    success

.. note::

   - ``period`` currently supports **0.001 to 1.0 s**.
   - ``loss`` is currently **not used**.

.. _SetVeljRt:

SetVeljRt
---------
Set the **global joint velocity limit** used by real-time servo motion.

**Request:**

.. code-block::

   float64[6] vel    # joint velocity limit [deg/s]

**Response:**

.. code-block::

   bool       success

.. note::

   If the measured velocity during servo motion exceeds the global limit, an **info message** is generated.

.. _SetVelxRt:

SetVelxRt
---------
Set the **global task velocity limit** used by real-time servo motion.

**Request:**

.. code-block::

   float64  trans     # task linear velocity limit [mm/s]
   float64  rotation  # task rotational velocity limit [deg/s]
                      # if set to the sentinel value -10000, it is auto-calculated from the linear velocity limit

**Response:**

.. code-block::

   bool     success

.. note::

   If the measured velocity during servo motion exceeds the global limit, an **info message** is generated.

.. _StartRtControl:

StartRtControl
--------------
Start sending/receiving the configured input/output data.

**Request:**

.. code-block::

   (None)

**Response:**

.. code-block::

   bool    success

.. _StopRtControl:

StopRtControl
-------------
Stop sending/receiving the configured input/output data.

**Request:**

.. code-block::

   (None)

**Response:**

.. code-block::

   bool    success

.. _WriteDataRt:

WriteDataRt
-----------
Write real-time **input** data (external controller → robot controller).  
Intended for sensors, DIO/AIO, and other commands from an external controller.

**Request:**

.. code-block::

   float64[6] external_force_torque
   int32      external_digital_input
   int32      external_digital_output
   float64[6] external_analog_input    # external analog input (6 channel)
   float64[6] external_analog_output   # external analog output (6 channel)

**Response:**

.. code-block::

   bool    success
