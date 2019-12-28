.. _install_v1.0:

================================
Install and Setup (v1.0)
================================

The recommended operating system is Ubuntu 16.04 LTS. *RoboSherlock* comes as a ROS stack, so you will need to install **ROS Kinetic** (desktop full). Installation instructions can be found on the ROS homepage_ and setup a catkin workspace as described here_.


.. _homepage: http://wiki.ros.org/ROS/Installation
.. _here: http://wiki.ros.org/catkin/Tutorials/create_a_workspace

These instructions are valid for the **devel branch** of the core package of RoboSherlock which you can get from the GitHub page: ::

    git clone https://github.com/RoboSherlock/robosherlock
    cd robosherlock
    git checkout tags/v1.0.0 -b v1.0.0
    git submodule init
    git submodule update    
   
Check out the repository into your catkin workspace. Before compiling you need to set up the dependencies for the project. 

Get dependencies
----------------

The following packages should be installed: ::
   
   sudo apt-get install libxerces-c-dev libicu-dev libapr1-dev mongodb openjdk-8-jdk ros-kinetic-libmongocxx-ros swi-prolog
   
   
.. warning:: RoboSherlock depends on algorithms implemented in OpenCV and PCL. For the current release we used the default versions that are included in ROS on Ubuntu 16.04. 
We also use C++11 features which might require compiling certain dependencies (if used from source) with C++11 support.


Set up Bash
-----------

It is recommended to add the [..]/robosherlock/scripts/ folder to your PATH. This way you can easily access some convenience scripts, for e.g. creating a new annotator, or a new ROS package that depends on RoboSherlock.

Compilation
-----------

You can either use `catkin_make` or catkin tools to compie robosherlock (and the rest of your workspace). 


Check out :ref:`pipeline` for details about how to run a small demo.


Installing and Running the query-answering
------------------------------------------

In order to use the query-answering capabilities of RoboSherlock to the fulles installing Knowrob is recommended. While basic query answering capabilities are working with the internal SWI-PROLOG engine, KnowrObr
offers more capabilities:

Knowrob: Follow the official Installation guide for the Knowrob Installation, which can be found on their installation page_.

.. _page: http://www.knowrob.org/installation


Check out :ref:`write_queries` for details about how to use the query-answering.
