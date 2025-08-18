.. _devcontainer:


Devcontainer Installation
=========================

This page explains how to set up and use a **Dev Container** with VS Code. It covers prerequisites, step‑by‑step usage, screenshots (placeholders included), common configuration files, and troubleshooting tips.

Overview
--------
A *Dev Container* lets you develop inside a Docker container with all tools preinstalled. VS Code connects seamlessly to the container, giving you a reproducible environment for your project.

Quick Start (VS Code)
---------------------
1) **Install Devcontainer Extensions in vscode**

   .. image:: images/devcontainer_cover.png
      :alt: .devcontainer folder structure
      :width: 700px
      :align: center

  .. raw:: html

      <br>
      <br>

2) **Open your project** in VS Code.

   .. code-block:: bash

      cd ros2_ws/src/doosan-robot2
      code .

   .. note::

      The Open Folder path must be the parent directory of .devcontainer (don’t open the .devcontainer folder itself).   

3) Ensure your repository contains a ``.devcontainer/`` folder with at least:

   - ``.devcontainer/devcontainer.json``
   - *(optional)* ``.devcontainer/Dockerfile``

   .. image:: images/devcontainer_02_folder.png
      :alt: .devcontainer folder structure
      :width: 700px
      :align: center

  .. raw:: html

      <br>
      <br>

4) Press **F1** (or ``Ctrl/Cmd+Shift+P``) and run:
   ``Dev Containers: Reopen in Container``

   .. image:: images/devcontainer_03_reopen.png
      :alt: Reopen in Container
      :width: 700px
      :align: center

  .. raw:: html

      <br>
      <br>


5) VS Code builds (or pulls) the image and starts the container. The first build can take some time.

   .. image:: images/devcontainer_04_building.png
      :alt: Building dev container
      :width: 700px
      :align: center

  .. raw:: html

      <br>
      <br>

GUI Apps on Linux (X11)
-----------------------
Some GUI apps (e.g., RViz2, Gazebo) need X11 access on Linux.

.. note::
   The Gazebo/RViz2 GUI may not appear due to missing X11 permissions
   (especially when using Docker or a remote session). Run the commands
   below **once per login (and after a reboot)** before launching.

   .. code-block:: shell

      xhost +                         # allow everyone; use only temporarily

   **These permissions reset after logout or reboot; re-apply as needed.**


References
----------
- `Dev Containers Documentation <https://code.visualstudio.com/docs/devcontainers/containers>`_
- `Docker Desktop / Engine <https://www.docker.com/products/docker-desktop/>`_

