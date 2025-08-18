.. _tcp_services:

tcp Services
============

.. contents::
   :depth: 1
   :local:

.. _ConfigCreateTcp:

ConfigCreateTcp
---------------

This is a service to register and use robot TCP information in advance for safety. 
TCP information registered using this service is stored in memory, so it must be set again after rebooting. 
if it is registered in the T/P application, it can be reused as it is added during the initialization process.

**Request:**

.. code-block::

   string          name         # tcp name 
   float64[6]      pos          # coordinates of the TCP 

**Response:**

.. code-block::

   bool success

.. _ConfigDeleteTcp:

ConfigDeleteTcp
---------------

It is a service for deleting the TCP information registered in advance in the robot controller.

**Request:**

.. code-block::

   string          name             # tcp name 

**Response:**

.. code-block::

   bool success

GetCurrentTcp
-------------

It is a service that fetches the currently set TCP information from the robot controller.

**Request:**

.. code-block::

   (None)

**Response:**

.. code-block::

   string         info # tcp name
   bool        success

.. _SetCurrentTcp:

SetCurrentTcp
-------------

It is a service that sets the information about the currently installed TCP.

**Request:**

.. code-block::

   string         name # tcp name

**Response:**

.. code-block::

   bool           success
