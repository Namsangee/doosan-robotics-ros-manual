.. _migration_note:

Migration from ROS 2 Humble to ROS 2 Jazzy
==========================================

This document summarizes the key changes when migrating from **ROS 2 Humble** to **ROS 2 Jazzy**.
For detailed, code-level guidance, refer to the relevant official migration guides in the ROS 2 documentation.

Bringup
-------

**MoveItConfigsBuilder Updates**

- Use ``.planning_pipelines()`` to explicitly specify ``planning_plugins``, ``default_planning_pipeline``, and ``load_all``.
- ``default_planning_pipeline`` is now **mandatory** — omitting it may result in runtime errors.
- After ``.to_moveit_configs()``, call ``.to_dict()`` to properly pass configuration to both ``MoveGroup`` and ``RViz`` nodes.

**Dynamic YAML Configuration**

- Introduce a new launch argument ``dynamic_yaml`` to generate controller configuration YAML dynamically at runtime.
- The system parses the ``robot_description`` to extract active/passive joint data and constructs controller YAML automatically.
- If a specific controller YAML (e.g. ``dsr_controller2_<model>.yaml``) is not found, it falls back to ``dsr_controller2.yaml``.

**Launch Flow Refactoring**

- Sequential node startup using ``OnProcessExit`` replaces previous parallel launch patterns.
- Recommended launch sequence::

    set_config_node → ros2_control_node → joint_state_broadcaster → dsr_controller2 → dsr_moveit_controller → move_group + RViz2

- ``set_config_node`` sets hardware parameters then triggers ``ros2_control_node``.
- ``joint_state_broadcaster`` now runs early to ensure ``/joint_states`` is available from the start.

Hardware Interface (``dsr_hw_interface2``)
------------------------------------------

**New Hardware-Centric Architecture**

- Migration from ``ControllerInterface`` to ``SystemInterface`` under Jazzy.
- Initialization steps (DRFL init, callback registration, parameter parsing) now reside in the hardware layer.
- Full support for lifecycle hooks: ``on_init()``, ``on_configure()``, ``on_activate()``.

**Callback Refactor**

- All monitoring callbacks (``OnMonitoringState``, ``OnMonitoringDataEx``, ``OnDisconnected``, ``OnLogAlarm``) are now managed in the hardware layer.
- These callbacks are registered in ``DRHWInterface::on_init()`` or ``on_activate()`` for consistency.

**Flexible DOF Handling & Parameters**

- Robot metadata (model name, degrees of freedom, gripper type) are parsed from ``HardwareInfo``.
- Arrays such as ``joint_position_``, ``joint_velocity_``, and ``command_`` are resized dynamically based on joint count.

**Hardware Mapping**

- ``hw_mapping_`` enables mapping between URDF/SRDF joint names and internal DRFL indices.
- Facilitates clarity and support for multiple robot configurations.

**Real-Time Control Enhancements**

- Use ``rt_host`` instead of ``host`` in DRCF versions ≥ 3.0.
- Functions like ``set_rt_control_output()`` and ``start_rt_control()`` execute only in **robot mode** (not emulator).

**Monitoring Upgrades**

- Replaced ``OnMonitoringDataCB`` with ``OnMonitoringDataExCB`` and ``OnMonitoringCtrlIOExCB``.
- Offers access to force/torque/position data in tool, world, and user frames — ideal for advanced compliance controls.

**Additional Improvements**

- Real-time error monitoring added via ``/error`` topic using ``LogAlarm`` callback.
- Emulator detection: automatic via loopback IP (``127.0.0.1``).

Xacro Fixes
-----------

- Define ``pi`` expressions with ``xacro:property`` at the top:

  .. code-block:: xml

     <xacro:property name="double_pi" value="${2.0 * pi}"/>
     <xacro:property name="neg_double_pi" value="${-2.0 * pi}"/>
     <xacro:property name="neg_pi" value="${-1.0 * pi}"/>

- Jazzy’s ``ros2_control`` cannot parse raw ``{2*pi}`` expressions.
- Always use ``${...}`` syntax and define via ``xacro:property``.

SRDF Fixes
----------

- Ensure ``<robot name="...">`` matches the URDF ``name``:

  .. code-block:: xml

     <!-- Before -->
     <robot name="dsr">

     <!-- After -->
     <robot name="m1013">

- Jazzy requires exact match between URDF and SRDF robot names.
- A mismatch will trigger an error::

     Semantic description is not specified for the same robot as the URDF

_planning.yaml Fixes (MoveIt2)
------------------------------

- Replace ``planning_plugin`` (string) with ``planning_plugins`` (array of strings).
- Separate adapters into ``request_adapters`` and ``response_adapters``:

  .. code-block:: yaml

     planning_plugins:
       - ompl_interface/OMPLPlanner

     request_adapters:
       - default_planning_request_adapters/ResolveConstraintFrames
       - default_planning_request_adapters/ValidateWorkspaceBounds
       - default_planning_request_adapters/CheckStartStateBounds
       - default_planning_request_adapters/CheckStartStateCollision

     response_adapters:
       - default_planning_response_adapters/AddTimeOptimalParameterization
       - default_planning_response_adapters/ValidateSolution
       - default_planning_response_adapters/DisplayMotionPath

- Lists must use a hyphen-prefixed format (``- item``) as ``string_array``.

QoS Changes
-----------

- Deprecated: ``rmw_qos_profile_*``
- Use ``rclcpp::QoS`` for all publishers and subscribers in C++:

  .. code-block:: cpp

     auto qos = rclcpp::QoS(10).best_effort();
     node->create_subscription<MsgType>("topic", qos, callback);

- Default QoS policy updates:
  - ``reliable`` → ``best_effort``
  - ``transient_local`` → ``volatile``

Summary of Official ``ros2_control`` Migration Highlights
----------------------------------------------------------

According to the official ROS 2 Jazzy migration guides and release notes:

**diff_drive_controller**
- ``cmd_vel`` must now use a *stamped* twist message.
- Deprecated parameters: ``has_velocity_limits``, ``has_acceleration_limits``, ``has_jerk_limits`` — set limits to ``.NAN`` instead.

**gripper_action_controller**
- Legacy controllers (``effort_...``, ``position_...``) removed. Use ``parallel_gripper_action_controller/GripperActionController``.

**joint_trajectory_controller**
- Default ``allow_nonzero_velocity_at_trajectory_end`` is now ``false``.
- ``start_with_holding`` removed; always holds on activation.
- Cancels goals on ``on_deactivate``.
- Discards empty trajectories.
- Angle wraparound auto-detected from URDF continuous joints; remove ``angle_wraparound`` parameter.
