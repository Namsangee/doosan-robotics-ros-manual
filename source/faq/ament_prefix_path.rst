.. _ament_prefix_path_issue:

AMENT_PREFIX_PATH Path Error
============================

This document outlines a common environment-related issue in ROS 2, where an incorrect or outdated `AMENT_PREFIX_PATH` can cause build or runtime errors. It also explains the root cause and provides multiple solutions.

1. Error Case
-------------

During workspace build or launch, you may encounter warnings or errors such as:

.. code-block:: bash

   Failed to find package: ...
   ament_cmake not found
   Resource not found for <package_name>

This typically happens when `AMENT_PREFIX_PATH` includes invalid or non-existent paths.

2. Root Cause Analysis
-----------------------

`AMENT_PREFIX_PATH` is an environment variable used by ROS 2 to locate installed packages.  
If this variable includes directories that no longer exist (e.g., deleted install folders), tools like `colcon build`, `ros2 launch`, or `ament_index_cpp` will fail to resolve dependencies correctly.

This issue often arises when:

- Switching between multiple workspaces
- Moving or deleting a workspace without updating environment variables
- Hardcoding outdated paths in `~/.bashrc` or `~/.zshrc`

3. Solution
-----------

To resolve the issue, you can either clean specific entries from the variable or reset the workspace environment completely.

**Option 1: Filter out broken paths**

.. code-block:: bash

   export AMENT_PREFIX_PATH=$(echo $AMENT_PREFIX_PATH | tr ':' '\n' | grep -v '/path/to/remove' | tr '\n' ':')

**Option 2: Reset the workspace environment**

.. code-block:: bash

   unset AMENT_PREFIX_PATH

   cd ~/ros2_ws  # or your workspace name
   rm -rf build/ install/ log/
   colcon build --symlink-install

   # Ensure dependencies are installed
   sudo apt update
   sudo apt install python3-ament-package

   rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
   source install/setup.bash

**Tip:** Avoid manually appending to `AMENT_PREFIX_PATH` in your shell configuration unless necessary.  
Use `source install/setup.bash` after each build to automatically configure paths.

