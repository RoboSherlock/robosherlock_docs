.. _install_rs:

=================
Install and Setup
=================

The recommended operating system is Ubuntu 16.04 LTS. *RoboSherlock* comes as a ROS package, so you will need to install **ROS Kinetic** (desktop full). Installation instructions can be found on the ROS homepage_ and setup a catkin workspace as described here_. Ubuntu 14.04 will still work, but as of now it is considered to be deprecated and support will stop soon. 

.. _homepage: http://wiki.ros.org/ROS/Installation
.. _here: http://wiki.ros.org/catkin/Tutorials/create_a_workspace

These instructions are valid for the core package of RoboSherlock which you can get from the GitHub page: ::

    git clone https://github.com/RoboSherlock/robosherlock
   
Check out the repository into your catkin workspace. Before compiling you need to set up the dependencies for the project. 

Get dependencies
----------------

Get robosherlock_msgs. This is a ROS package and needs to be checked out in your catkin workspace. ::

	git clone https://github.com/RoboSherlock/robosherlock_msgs

The following packages should be installed (this will add the RoboSherlock PPA to your software sources): ::
   
   sudo add-apt-repository ppa:robosherlock/ppa
   sudo apt-get update
   sudo apt-get install rapidjson automake libxerces-c-dev libicu-dev libapr1-dev mongodb openjdk-8-jdk ros-kinetic-libmongocxx-ros
   
   
.. warning:: RoboSherlock heavily depends on algorithms implemented in OpenCV and PCL. For the current release we used the default versions that are included in ROS on Ubuntu 14.04 or Ubuntu 16.04. We also use C++11 features which might require compiling certain dependencies with C++11 support (e.g. libmongocxx-ros on 14.04)

Get *uimacpp* and install to */usr/local* or any other folder that is in your LD_LIBRARY_PATH and PATH. Uimacpp expects the Java headers in */usr/lib/jvm/java-[version]-openjdk-amd64/include*, so depending on your OS you might need to create symlinks for the header files located in the */usr/lib/jvm/java-8-openjdk-amd64/include/linux* (i.e. java 7 and 6 come with symlinks 8 and 9 don't). In the command below replace the version of OpenJdk with the one you have installed::
  
   git clone https://github.com/robosherlock/uima-uimacpp.git uimacpp
   cd uimacpp
   ./autogen.sh
   ./configure --without-activemq --with-jdk=/usr/lib/jvm/java-7-openjdk-amd64/include --prefix=/usr/local --with-icu=/usr
   make
   sudo make install

If all went correct */usr/local/lib* will contain *libuima.so*.

Experimental for 16.04: ROS package uimacpp installation (from source):
-----------------------------------------------------------------------

First go to src folder of your catkin workspace, and then::

  git clone https://github.com/RoboSherlock/uimacpp_ros.git

For now, the corresponding adapted RoboSherlock can be downloaded by doing::

  git clone -b uima-ros https://github.com/MaidouPP/robosherlock.git

Pending a pull request and more thorough tests this will be incorproated in the main repo soon. 


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


Check out :ref:`pipeline` 
for details about how to run a small demo.


Installing and Running the query-answering
------------------------------------------

In order to use the query-answering capabilities of RoboSherlock, the following libraries need to be installed:

Knowrob: Follow the official Installation guide for the Knowrob Installation, which can be found on their installation page_.

.. _page: http://www.knowrob.org/installation

robosherlock_knowrob: Check out the repository into your workspace. ::

   git clone https://github.com/RoboSherlock/robosherlock_knowrob

Check out :ref:`write_queries` for details about how to use the query-answering.
