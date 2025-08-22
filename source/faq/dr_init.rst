.. _dr_init:

DR_init Import Issue
====================

If a dr_init error occurs even after a successful build, please refer to the following information.
This document describes a common error related to the `DR_init` module in the Doosan ROS 2 environment, including the root cause and its resolution.

1. Error Case
-------------

When running a Python script such as `dance_m1013.py`, the following error occurs:

.. code-block:: bash

   $:~/ros2_ws/src/doosan-robot2/dsr_example2/dsr_example/dsr_example/demo$ python3 dance_m1013.py
   ModuleNotFoundError: No module named 'DR_init'

This indicates that Python is unable to locate the `DR_init` module.

2. Root Cause Analysis
-----------------------

The error is caused by the `PYTHONPATH` not including the installation path of the `DR_init.py` file.  
By default, ROS 2 installs this module inside the `install/dsr_common2/lib/dsr_common2/imp/` directory, which is not automatically included in the Python import path.

3. Solution
-------------------------

To resolve the issue, explicitly set the `PYTHONPATH` environment variable to include the location of `DR_init.py`.

**Step-by-step instructions:**

.. code-block:: bash

   # Step 1: Locate the module
   find ~/ros2_ws -name "DR_init.py"

   # Step 2: Set the PYTHONPATH
   export PYTHONPATH=~/ros2_ws/install/dsr_common2/lib/dsr_common2/imp

   # Step 3: Source the changes
   source ~/.bashrc

   # (Optional) Check if PYTHONPATH is set correctly
   echo $PYTHONPATH

   # (Optional) Reset PYTHONPATH if needed
   unset PYTHONPATH

**Note:** You can also add the `export PYTHONPATH=...` line to your `~/.bashrc` or `~/.zshrc` for persistent usage across terminals.


This should ensure that the Python runtime correctly resolves the `DR_init` module and prevents import errors.
