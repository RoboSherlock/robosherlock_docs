.. _install_rs:
=================
Install and Setup
=================

.. note:: The installation instructions are for the core package of *RoboSherlock*. This does **NOT** contain the knowledge-based reasoning mechanisms and the question-answering. These will be released separately, in order to keep dependencies of the core package at a minimum.

The recommended operating system is Ubuntu 14.04 LTS 64bit. *RoboSherlock* comes as a ROS package, so you will need to install **ROS Indigo** (desktop full). Installation instructions can be found on the ROS homepage_ and setup a catkin workspace as described here_.

.. _homepage: http://wiki.ros.org/indigo/Installation
.. _here: http://wiki.ros.org/catkin/Tutorials/create_a_workspace

These instructions are valid for the core package of RoboSherlock which you can get from the GitHub page: ::

    git clone https://github.com/RoboSherlock/robosherlock
   
Check out the repository into your catkin workspace. Before compiling you need to set up the dependencies for the project. 

Get dependencies
----------------

The following packages should be installed: ::
   
   sudo apt-get install automake libxerces-c-dev libicu-dev libapr1-dev mongodb scons openjdk-7-jdk

Install the necessary ros packages: ::

   sudo apt-get install ros-indigo-openni-camera ros-indigo-openni-launch
   
.. warning:: RoboSherlock heavily depends on algorithms implemented in OpenCV and PCL. For the current release we used the default versions that are also included in ROS Indigo and Ubuntu 14.04, namely ``OpenCV 2.4.8`` and ``PCL 1.7.2``

Get *uimacpp* and install to */usr/local* or any other folder that is in your LD_LIBRARY_PATH and PATH::
  
   git clone https://github.com/robosherlock/uima-uimacpp.git uimacpp
   cd uimacpp
   ./autogen.sh
   ./configure --without-activemq --with-jdk=/usr/lib/jvm/java-7-openjdk-amd64/include --prefix=/usr/local --with-icu=/usr
   make
   sudo make install

If all went correct */usr/local/lib* will contain *libuima.so*.

Get mongo-cxx-driver (*branch 26compat*) and install to /usr/local::
   
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

   export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH}

It is recommended to add the [..]/robosherlock/scripts/ folder to your PATH. This way you can easily access
some convenience scripts, for e.g. creating a new annotator, or a new ROS package that depends on RoboSherlock.

Compilation
-----------

You can either use `catkin_make` or the *rsmake* build script provided in *robosherlock/scripts/* to compie robosherlock (and the rest of your workspace). Passing one of *deb, reb, rel* will build your catkin workspace in debug, release with debug symbols or release.

.. symbolic link info for having the different builds in parallel and activating the one or the other using rsmake xxx



Check out :ref:`pipeline` 
for details about how to run a small demo.


