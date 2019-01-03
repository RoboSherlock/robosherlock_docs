.. _install_devel_rs:

================================
Install and Setup (devel branch)
================================

The recommended operating system is Ubuntu 16.04 LTS. *RoboSherlock* comes as a ROS stack, so you will need to install **ROS Kinetic** (desktop full). Installation instructions can be found on the ROS homepage_ and setup a catkin workspace as described here_.


.. _homepage: http://wiki.ros.org/ROS/Installation
.. _here: http://wiki.ros.org/catkin/Tutorials/create_a_workspace

These instructions are valid for the **devel branch** of the core package of RoboSherlock which you can get from the GitHub page: ::

    git clone https://github.com/RoboSherlock/robosherlock -b devel --recursive
   
Check out the repository into your catkin workspace. Before compiling you need to set up the dependencies for the project. 

Get dependencies
----------------

The following packages should be installed: ::
   
   sudo apt-get install libxerces-c-dev libicu-dev libapr1-dev mongodb openjdk-8-jdk ros-kinetic-libmongocxx-ros
   
   
.. warning:: RoboSherlock heavily depends on algorithms implemented in OpenCV and PCL. For the current release we used the default versions that are included in ROS on Ubuntu 14.04 or Ubuntu 16.04. We also use C++11 features which might require compiling certain dependencies with C++11 support


Set up Bash
-----------

Put the right paths into your ~/.bashrc.::

   export APR_HOME=/usr
   export ICU_HOME=/usr
   export XERCES_HOME=/usr

   export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH}

It is recommended to add the [..]/robosherlock/scripts/ folder to your PATH. This way you can easily access some convenience scripts, for e.g. creating a new annotator, or a new ROS package that depends on RoboSherlock.

Compilation
-----------

You can either use `catkin_make` or catkin tools to compie robosherlock (and the rest of your workspace). 


Check out :ref:`pipeline` for details about how to run a small demo.


Installing and Running the query-answering
------------------------------------------

In order to use the query-answering capabilities of RoboSherlock:

Knowrob: Follow the official Installation guide for the Knowrob Installation, which can be found on their installation page_.

.. _page: http://www.knowrob.org/installation

robosherlock_knowrob: Check out the repository into your workspace. ::

   git clone https://github.com/RoboSherlock/robosherlock_knowrob -b dev_planner

Check out :ref:`write_queries` for details about how to use the query-answering.
