.. _install_rs:
=================
Install and Setup
=================

Recommended operating system is Ubuntu 14.04 LTS 64bit. The framew
RoboSherlock comes as a ROS package, so you will need to install **ROS Indigo** (desktop full). 
Installation instructions can be found on the ROS homepage_ and setup a catkin workspace as described here_.

.. _homepage: http://wiki.ros.org/indigo/Installation
.. _here: http://wiki.ros.org/catkin/Tutorials/create_a_workspace

These instructions are valid for the core package of RoboSherlock which you can get from the project
GitHub page: ::

    git clone https://github.com/RoboSherlock/robosherlock
   
Check the repository out in your catkin workspace. Before compiling you need to set up the dependencies 
for the project. 

Get dependencies
----------------

The following packages should be installed::
   
   sudo apt-get install automake libxerces-c-dev libicu-dev openjdk-7-jdk libapr1-dev libgphoto2-2-dev mongodb libhdf5-serial-dev scons

Install the necessary ros packages.::

   sudo apt-get install ros-indigo-openni-camera ros-indigo-openni-launch

Get *uimacpp* and install to */usr/local* or any other folder that is in your LD_LIBRARY_PATH and PATH::
  
   git clone https://github.com/bbferka/uima-uimacpp.git uimacpp
   cd uimacpp
   ./autogen.sh
   ./configure --without-activemq --with-jdk=/usr/lib/jvm/java-7-openjdk-amd64/include --prefix=/usr/local --with-icu=/usr
   make
   sudo make install

If all went correct *ls /usr/local/lib* will contain *libuima.so*.

Get mongo-cxx-driver and install to /usr/local::
   
   git clone https://github.com/mongodb/mongo-cxx-driver.git
   cd mongo-cxx-driver/
   git checkout 26compat 
   sudo scons --full --use-system-boost --prefix=/usr/local --ssl --sharedclient install-mongoclient   

Set up Bash
-----------

Put the right paths into your ~/.bashrc.::

   export APR_HOME=/usr
   export ICU_HOME=/usr
   export XERCES_HOME=/usr

   export JAVA_HOME=/usr/lib/jvm/java-7-openjdk-amd64/
   export JAVA_INCLUDE=${JAVA_HOME}/include

   export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH}

You are ready to compile with `catkin_make`. Check out :ref:`pipeline` 
for details about how to run the a small demo.

It is recommended to add the [..]/robosherlock/scripts/ folder to your PATH. This way you can easily access
some convenience scripts, for e.g. creating a new annotator, or a new ROS package that depends on RoboSherlock.


