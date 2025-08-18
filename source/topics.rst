.. _topics:

Topics
******

This section describes the various ROS2 Topics used in the Doosan Robotics package, detailing the structure of their messages.

.. contents::
   :depth: 2
   :local:

RobotState
==========

Provides the current state of the robot.

.. code-block::
   :caption: dsr_msgs2/msg/RobotState.msg

   # timestamp at the data of data acquisition
   float64                      time_stamp
   # actual joint position from incremental encoder at motor side(used for control) [deg]
   float64[6]                   actual_joint_position
   # actual joint position from absolute encoder at link side (used for exact link position) [deg]
   float64[6]                   actual_joint_position_abs
   # actual joint velocity from incremental encoder at motor side [deg/s]
   float64[6]                   actual_joint_velocity
   # actual joint velocity from absolute encoder at link side [deg/s]
   float64[6]                   actual_joint_velocity_abs
   # actual robot tcp position w.r.t. base coordinates: (x, y, z, a, b, c), where (a, b, c) follows Euler ZYZ notation [mm, deg]
   float64[6]                   actual_tcp_position
   # actual robot tcp velocity w.r.t. base coordinates [mm, deg/s]
   float64[6]                   actual_tcp_velocity
   # actual robot flange position w.r.t. base coordinates: (x, y, z, a, b, c), where (a, b, c) follows Euler ZYZ notation [mm, deg]
   float64[6]                   actual_flange_position
   # robot flange velocity w.r.t. base coordinates [mm, deg/s]
   float64[6]                   actual_flange_velocity
   # actual motor torque applying gear ratio = gear_ratio * current2torque_constant * motor current [Nm]
   float64[6]                   actual_motor_torque
   # estimated joint torque by robot controller [Nm]
   float64[6]                   actual_joint_torque
   # calibrated joint torque sensor data [Nm]
   float64[6]                   raw_joint_torque
   # calibrated force torque sensor data w.r.t. flange coordinates [N, Nm]
   float64[6]                   raw_force_torque
   # estimated external joint torque [Nm]
   float64[6]                   external_joint_torque
   # estimated tcp force w.r.t. base coordinates [N, Nm] 
   float64[6]                   external_tcp_force
   # target joint position [deg]
   float64[6]                   target_joint_position
   # target joint velocity [deg/s]
   float64[6]                   target_joint_velocity
   # target joint acceleration [deg/s^2] 
   float64[6]                   target_joint_acceleration
   # target motor torque [Nm] 
   float64[6]                   target_motor_torque
   # target tcp position w.r.t. base coordinates: (x, y, z, a, b, c), where (a, b, c) follows Euler ZYZ notation [mm, deg] 
   float64[6]                   target_tcp_position
   # target tcp velocity w.r.t. base coordinates [mm, deg/s]
   float64[6]                   target_tcp_velocity
   # jacobian matrix=J(q) w.r.t. base coordinates
   std_msgs/Float64MultiArray[] jacobian_matrix
   # gravity torque=g(q) [Nm]
   float64[6]                   gravity_torque
   # coriolis matrix=C(q,q_dot)  [6][6]
   std_msgs/Float64MultiArray[] coriolis_matrix
   # mass matrix=M(q) [6][6]
   std_msgs/Float64MultiArray[] mass_matrix
   # robot configuration 
   uint16                       solution_space
   # minimum singular value 
   float64                      singularity
   # current operation speed rate(1~100 %) 
   float64                      operation_speed_rate
   # joint temperature(celsius) 
   float64[6]                   joint_temperature
   # controller digital input(16 channel) 
   uint16                       controller_digital_input
   # controller digital output(16 channel) 
   uint16                       controller_digital_output
   # controller analog input type(2 channel) 
   uint8[2]                      controller_analog_input_type
   # controller analog input(2 channel) 
   float64[2]                   controller_analog_input
   # controller analog output type(2 channel) 
   uint8[2]                     controller_analog_output_type
   # controller analog output(2 channel) 
   float64[2]                   controller_analog_output
   # flange digital input(A-Series: 2 channel, M/H-Series: 6 channel) 
   uint8                        flange_digital_input
   # flange digital output(A-Series: 2 channel, M/H-Series: 6 channel) 
   uint8                        flange_digital_output
   # flange analog input(A-Series: 2 channel, M/H-Series: 4 channel) 
   float64[4]                   flange_analog_input
   # strobe count(increased by 1 when detecting setting edge) 
   uint8[2]                     external_encoder_strobe_count
   # external encoder count 
   uint16[2]                    external_encoder_count
   # final goal joint position (reserved) 
   float64[6]                   goal_joint_position
   # final goal tcp position (reserved) 
   float64[6]                   goal_tcp_position
   # ROBOT_MODE_MANUAL(0), ROBOT_MODE_AUTONOMOUS(1), ROBOT_MODE_MEASURE(2) 
   uint8                        robot_mode
   # STATE_INITIALIZING(0), STATE_STANDBY(1), STATE_MOVING(2), STATE_SAFE_OFF(3), STATE_TEACHING(4), STATE_SAFE_STOP(5), STATE_EMERGENCY_STOP, STATE_HOMMING, STATE_RECOVERY, STATE_SAFE_STOP2, STATE_SAFE_OFF2, 
   uint8                        robot_state
   # position control mode, torque mode 
   uint16                       control_mode
   # Reserved 
   uint8[256]                   reserved

RobotStop
=========

Provides information about the robot's stop mode.

.. code-block::
   :caption: dsr_msgs2/msg/RobotStop.msg

   #____________________________________________________________________________________________
   # [ robot stop mode ] 
   # 0 : STOP_TYPE_QUICK_STO
   # 1 : STOP_TYPE_QUICK
   # 2 : STOP_TYPE_SLOW
   # 3 : STOP_TYPE_HOLD = STOP_TYPE_EMERGENCY
   #
   #____________________________________________________________________________________________

   int32 stop_mode  

RobotError
==========

Provides information about robot errors.

.. code-block::
   :caption: dsr_msgs2/msg/RobotError.msg

   #____________________________________________________________________________________________
   # [ robot error msg ] 
   #____________________________________________________________________________________________

   int32    level   # INFO =1, WARN =2, ERROR =3 
   int32    group   # SYSTEM =1, MOTION =2, TP =3, INVERTER =4, SAFETY_CONTROLLER =5   
   int32    code    # error code 
   string    msg1    # error msg 1
   string    msg2    # error msg 2
   string    msg3    # error msg 3

LogAlarm
========

Provides log information for alarms.

.. code-block::
   :caption: dsr_msgs2/msg/LogAlarm.msg

   #____________________________________________________________________________________________
   # log of alarm
   #____________________________________________________________________________________________

   int32         level
   int32         group
   int32         index
   string[3]     param

ModbusState
===========

Provides the state of Modbus signals.

.. code-block::
   :caption: dsr_msgs2/msg/ModbusState.msg

   #____________________________________________________________________________________________
   #Custom msg for RobotState.msg -- MAX_SIZE = 100
   #____________________________________________________________________________________________

   string  modbus_symbol    # Modbus Signal Name
   int32   modbus_value     # Modbus Register Value (Unsigned : 0 ~ 65535)

JogMultiAxis
============

Provides information for multi-axis jogging.

.. code-block::
   :caption: dsr_msgs2/msg/JogMultiAxis.msg

   #____________________________________________________________________________________________
   # multi jog
   # multi jog speed = (250mm/s x 1.73) x unit vecter x speed [%] 
   #____________________________________________________________________________________________

   float64[6]  jog_axis          # unit vecter of Task space [Tx, Ty, Tz, Rx, Ry, Rz] : -1.0 ~ +1.0 
   int8        move_reference    # 0 : MOVE_REFERENCE_BASE, 1 : MOVE_REFERENCE_TOOL, 2 : MOVE_REFERENCE_WORLD
   float64     speed             # jog speed [%]

ServojRtStream
==============

Provides real-time stream data for Servoj.

.. code-block::
   :caption: dsr_msgs2/msg/ServojRtStream.msg

   #____________________________________________________________________________________________
   # servoj_rt
   # 
   #____________________________________________________________________________________________

   float64[6] pos               # position  
   float64[6] vel               # velocity
   float64[6] acc               # acceleration
   float64    time              # time

ServojStream
============

Provides stream data for Servoj.

.. code-block::
   :caption: dsr_msgs2/msg/ServojStream.msg

   #____________________________________________________________________________________________
   # servoj
   # 
   #____________________________________________________________________________________________

   float64[6] pos               # position  
   float64[6] vel               # velocity
   float64[6] acc               # acceleration
   float64    time              # time
   int8       mode              # servoj mode; 0:DR_SERVO_OVERRIDE, 1:DR_SERVO_QUEUE

ServolRtStream
==============

Provides real-time stream data for Servol.

.. code-block::
   :caption: dsr_msgs2/msg/ServolRtStream.msg

   #____________________________________________________________________________________________
   # servol_rt
   # 
   #____________________________________________________________________________________________

   float64[6] pos               # position  
   float64[6] vel               # velocity
   float64[6] acc               # acceleration
   float64    time              # time

ServolStream
============

Provides stream data for Servol.

.. code-block::
   :caption: dsr_msgs2/msg/ServolStream.msg

   #____________________________________________________________________________________________
   # servol
   # 
   #____________________________________________________________________________________________

   float64[6] pos               # position  
   float64[2] vel               # velocity
   float64[2] acc               # acceleration
   float64    time              # time

SpeedjRtStream
==============

Provides real-time stream data for Speedj.

.. code-block::
   :caption: dsr_msgs2/msg/SpeedjRtStream.msg

   #____________________________________________________________________________________________
   # speedj_rt
   # 
   #____________________________________________________________________________________________

   float64[6] vel               # velocity
   float64[6] acc               # acceleration
   float64    time              # time

SpeedjStream
============

Provides stream data for Speedj.

.. code-block::
   :caption: dsr_msgs2/msg/SpeedjStream.msg

   #____________________________________________________________________________________________
   # speedj
   # 
   #____________________________________________________________________________________________

   float64[6] vel               # velocity
   float64[6] acc               # acceleration
   float64    time              # time

SpeedlRtStream
==============

Provides real-time stream data for Speedl.

.. code-block::
   :caption: dsr_msgs2/msg/SpeedlRtStream.msg

   #____________________________________________________________________________________________
   # speedl_rt
   # 
   #____________________________________________________________________________________________

   float64[6] vel               # velocity
   float64[6] acc               # acceleration
   float64    time              # time

SpeedlStream
============

Provides stream data for Speedl.

.. code-block::
   :caption: dsr_msgs2/msg/SpeedlStream.msg

   #____________________________________________________________________________________________
   # speedl
   # 
   #____________________________________________________________________________________________

   float64[6] pos               # position  
   float64[2] vel               # velocity
   float64[2] acc               # acceleration
   float64    time              # time

TorqueRtStream
==============

Provides real-time stream data for Torque.

.. code-block::
   :caption: dsr_msgs2/msg/TorqueRtStream.msg

   #____________________________________________________________________________________________
   # torque_rt
   # 
   #____________________________________________________________________________________________

   float64[6] tor               # motor torque
   float64    time              # time

AlterMotionStream
=================

Provides stream data for AlterMotion.

.. code-block::
   :caption: dsr_msgs2/msg/AlterMotionStream.msg

   #____________________________________________________________________________________________
   # alter_motion  
   # 
   #____________________________________________________________________________________________

   float64[6] pos               # position  

RobotStateRt
============

Provides real-time robot state data.

.. code-block::
   :caption: dsr_msgs2/msg/RobotStateRt.msg

   # timestamp at the data of data acquisition
   float64                      time_stamp
   # actual joint position from incremental encoder at motor side(used for control) [deg]
   float64[6]                   actual_joint_position
   # actual joint position from absolute encoder at link side (used for exact link position) [deg]
   float64[6]                   actual_joint_position_abs
   # actual joint velocity from incremental encoder at motor side [deg/s]
   float64[6]                   actual_joint_velocity
   # actual joint velocity from absolute encoder at link side [deg/s]
   float64[6]                   actual_joint_velocity_abs
   # actual robot tcp position w.r.t. base coordinates: (x, y, z, a, b, c), where (a, b, c) follows Euler ZYZ notation [mm, deg]
   float64[6]                   actual_tcp_position
   # actual robot tcp velocity w.r.t. base coordinates [mm, deg/s]
   float64[6]                   actual_tcp_velocity
   # actual robot flange position w.r.t. base coordinates: (x, y, z, a, b, c), where (a, b, c) follows Euler ZYZ notation [mm, deg]
   float64[6]                   actual_flange_position
   # robot flange velocity w.r.t. base coordinates [mm, deg/s]
   float64[6]                   actual_flange_velocity
   # actual motor torque applying gear ratio = gear_ratio * current2torque_constant * motor current [Nm]
   float64[6]                   actual_motor_torque
   # estimated joint torque by robot controller [Nm]
   float64[6]                   actual_joint_torque
   # calibrated joint torque sensor data [Nm]
   float64[6]                   raw_joint_torque
   # calibrated force torque sensor data w.r.t. flange coordinates [N, Nm]
   float64[6]                   raw_force_torque
   # estimated external joint torque [Nm]
   float64[6]                   external_joint_torque
   # estimated tcp force w.r.t. base coordinates [N, Nm] 
   float64[6]                   external_tcp_force
   # target joint position [deg]
   float64[6]                   target_joint_position
   # target joint velocity [deg/s]
   float64[6]                   target_joint_velocity
   # target joint acceleration [deg/s^2] 
   float64[6]                   target_joint_acceleration
   # target motor torque [Nm] 
   float64[6]                   target_motor_torque
   # target tcp position w.r.t. base coordinates: (x, y, z, a, b, c), where (a, b, c) follows Euler ZYZ notation [mm, deg] 
   float64[6]                   target_tcp_position
   # target tcp velocity w.r.t. base coordinates [mm, deg/s]
   float64[6]                   target_tcp_velocity
   # jacobian matrix=J(q) w.r.t. base coordinates
   std_msgs/Float64MultiArray[] jacobian_matrix
   # gravity torque=g(q) [Nm]
   float64[6]                   gravity_torque
   # coriolis matrix=C(q,q_dot)  [6][6]
   std_msgs/Float64MultiArray[] coriolis_matrix
   # mass matrix=M(q) [6][6]
   std_msgs/Float64MultiArray[] mass_matrix
   # robot configuration 
   uint16                       solution_space
   # minimum singular value 
   float64                      singularity
   # current operation speed rate(1~100 %) 
   float64                      operation_speed_rate
   # joint temperature(celsius) 
   float64[6]                   joint_temperature
   # controller digital input(16 channel) 
   uint16                       controller_digital_input
   # controller digital output(16 channel) 
   uint16                       controller_digital_output
   # controller analog input type(2 channel) 
   uint8[2]                      controller_analog_input_type
   # controller analog input(2 channel) 
   float64[2]                   controller_analog_input
   # controller analog output type(2 channel) 
   uint8[2]                     controller_analog_output_type
   # controller analog output(2 channel) 
   float64[2]                   controller_analog_output
   # flange digital input(A-Series: 2 channel, M/H-Series: 6 channel) 
   uint8                        flange_digital_input
   # flange digital output(A-Series: 2 channel, M/H-Series: 6 channel) 
   uint8                        flange_digital_output
   # flange analog input(A-Series: 2 channel, M/H-Series: 4 channel) 
   float64[4]                   flange_analog_input
   # strobe count(increased by 1 when detecting setting edge) 
   uint8[2]                     external_encoder_strobe_count
   # external encoder count 
   uint16[2]                    external_encoder_count
   # final goal joint position (reserved) 
   float64[6]                   goal_joint_position
   # final goal tcp position (reserved) 
   float64[6]                   goal_tcp_position
   # ROBOT_MODE_MANUAL(0), ROBOT_MODE_AUTONOMOUS(1), ROBOT_MODE_MEASURE(2) 
   uint8                        robot_mode
   # STATE_INITIALIZING(0), STATE_STANDBY(1), STATE_MOVING(2), STATE_SAFE_OFF(3), STATE_TEACHING(4), STATE_SAFE_STOP(5), STATE_EMERGENCY_STOP, STATE_HOMMING, STATE_RECOVERY, STATE_SAFE_STOP2, STATE_SAFE_OFF2, 
   uint8                        robot_state
   # position control mode, torque mode 
   uint16                       control_mode
   # Reserved 
   uint8[256]                   reserved

RobotDisconnection
==================

Event driven when the robot connection losts.

.. code-block::
   :caption: dsr_msgs2/msg/RobotDisconnection.msg

   ### Event driven when the robot connection losts.
