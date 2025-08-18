.. _python_api:

Python API
==========


This page provides a categorized overview of available Python API functions for controlling and monitoring Doosan Robots in ROS 2 environments.

The functions are grouped by their purpose—such as motion control, auxiliary functions, force control, I/O operations, DRL execution, and real-time streaming—so developers can quickly find and utilize the appropriate commands in their applications.

Each function listed here can be imported from ``DSR_ROBOT2`` in a ROS2 node, and most require that the robot be properly connected and in the correct mode.

Refer to :ref:`DSR_ROBOT2 library tutorial <dsr_robot_tutorial>` for usage.


.. list-table::
   :widths: 30 80
   :header-rows: 1

   * - Category
     - Functions

   * - **System Operations**
     - - set_robot_mode()
       - get_robot_mode()
       - set_robot_system()
       - get_robot_system()
       - get_robot_state()
       - set_robot_speed_mode()
       - get_robot_speed_mode()
       - set_safe_stop_reset_type()
       - get_last_alarm()
       - get_current_pose()

   * - **Standard Motion (Service-based)**
     - - movej()
       - movel()
       - movejx()
       - movec()
       - movesj()
       - movesx()
       - moveb()
       - move_spiral()
       - move_periodic()
       - move_wait()
       - jog()
       - jog_multi()
       - trans()
       - fkin()
       - ikin()
       - set_ref_coord()
       - move_home()
       - check_motion()
       - change_operation_speed()
       - enable_alter_motion()
       - alter_motion()
       - disable_alter_motion()
       - set_singularity_handling()

   * - **Real-time & Streaming Motion (Topic-based)**
     - - servoj()
       - servol()
       - speedj()
       - speedl()
       - servoj_rt()
       - servol_rt()
       - speedj_rt()
       - speedl_rt()
       - torque_rt()
       - alter_motion_stream()

   * - **Auxiliary Control Operations**
     - - get_control_mode()
       - get_control_space()
       - get_current_posj()
       - get_current_velj()
       - get_desired_posj()
       - get_desired_velj()
       - get_current_posx()
       - get_current_velx()
       - get_desired_posx()
       - get_desired_velx()
       - get_current_tool_flange_posx()
       - get_current_solution_space()
       - get_current_rotm()
       - get_joint_torque()
       - get_external_torque()
       - get_tool_force()
       - get_solution_space()
       - get_orientation_error()

   * - **Force/Stiffness Control**
     - - get_workpiece_weight()
       - reset_workpiece_weight()
       - parallel_axis1()
       - parallel_axis2()
       - align_axis1()
       - align_axis2()
       - is_done_bolt_tightening()
       - release_compliance_ctrl()
       - task_compliance_ctrl()
       - set_stiffnessx()
       - calc_coord()
       - set_user_cart_coord1()
       - set_user_cart_coord2()
       - set_user_cart_coord3()
       - overwrite_user_cart_coord()
       - get_user_cart_coord()
       - set_desired_force()
       - release_force()
       - check_position_condition()
       - check_force_condition()
       - check_orientation_condition1()
       - check_orientation_condition2()
       - coord_transform()

   * - **GPIO**
     - - set_digital_output()
       - get_digital_input()
       - set_tool_digital_output()
       - get_tool_digital_input()
       - set_analog_output()
       - get_analog_input()
       - set_mode_analog_output()
       - set_mode_analog_input()
       - get_digital_output()
       - get_tool_digital_output()

   * - **Modbus**
     - - set_modbus_output()
       - get_modbus_input()
       - add_modbus_signal()
       - del_modbus_signal()

   * - **TCP (Tool Center Point) Operations**
     - - set_current_tcp()
       - get_current_tcp()
       - config_create_tcp()
       - config_delete_tcp()

   * - **Tool Operations**
     - - set_current_tool()
       - get_current_tool()
       - config_create_tool()
       - config_delete_tool()
       - set_tool_shape()

   * - **DRL (Doosan Robot Language) Operations**
     - - drl_pause()
       - drl_resume()
       - drl_start()
       - drl_stop()
       - get_drl_state()

   * - **Real-time (RT) Control**
     - - connect_rt_control()
       - disconnect_rt_control()
       - get_rt_control_output_version_list()
       - get_rt_control_input_version_list()
       - get_rt_control_input_data_list()
       - get_rt_control_output_data_list()
       - start_rt_control()
       - stop_rt_control()
       - set_rt_control_input()
       - set_rt_control_output()
       - set_velj_rt()
       - set_accj_rt()
       - set_velx_rt()
       - set_accx_rt()
       - read_data_rt()
       - write_data_rt()
