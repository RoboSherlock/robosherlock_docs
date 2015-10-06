.. _install_rs:
=================
Install and Setup
=================

Recommended operating system is Ubuntu 14.04 LTS 64bit.

Install **ROS Indigo** (desktop full). Installation instructions can be found on the ROS homepage_ and setup a catkin workspace as described here_.

.. _homepage: http://wiki.ros.org/indigo/Installation
.. _here: http://wiki.ros.org/catkin/Tutorials/create_a_workspace

Using these instructions does not guarantee that all components will work out of the box. Please check the :ref:`Overview <overview_rs>` section for more detail.

Get dependencies
----------------

- The following packages should be installed::
   
   sudo apt-get install automake libxerces-c-dev libicu-dev openjdk-7-jdk libapr1-dev libgphoto2-2-dev mongodb libhdf5-serial-dev libcppnetlib-dev protobuf-compiler scons
  
- Install the necessary ros packages.::

   sudo apt-get install ros-indigo-openni-camera ros-indigo-openni-launch

- For **ROS Indigo** there is no released rosjava package at the moment. You can install it from source following these instructions_.
.. _instructions: http://wiki.ros.org/rosjava/Tutorials/indigo/Installation

- get *uimacpp* and install to */usr/local* or any other folder that is in your LD_LIBRARY_PATH and PATH::
  
   git clone https://github.com/bbferka/uima-uimacpp.git uimacpp
   cd uimacpp
   ./autogen.sh
   ./configure --without-activemq --with-jdk=/usr/lib/jvm/java-7-openjdk-amd64/include --prefix=/usr/local --with-icu=/usr
   make
   sudo make install

If all went correct *ls /usr/local/lib* will contain *libuima.so*.

- get mongo-cxx-driver and install to /usr/local::
   
   git clone https://github.com/mongodb/mongo-cxx-driver.git
   cd mongo-cxx-driver/
   git checkout 26compat 
   sudo scons --full --use-system-boost --prefix=/usr/local --ssl --sharedclient install-mongoclient

- Get the UIMA Java SDK Binaries. Download and extract latest binaries to a folder of your choice::

   wget http://mirror.softaculous.com/apache//uima//uimaj-2.6.0/uimaj-2.6.0-bin.tar.gz
   tar -xf uimaj-2.6.0-bin.tar.gz
	
- Checkout dependencies in your catkin workspace.::

   git clone https://github.com/code-iai/iai_common_msgs.git
   git clone https://github.com/code-iai/designator_integration.git
   git clone https://github.com/code-iai/iai_photo.git

Set up Bash
-----------

- Put the right paths into your ~/.bashrc. In the following code, replace ''${PATH_TO_UIMA/apache-uima}'' with the location where you extracted the apache-uima lib.::

   export UIMA_HOME=${PATH_TO_UIMA}/apache-uima
   export UIMA_HOME_LIBS=${UIMA_HOME}/lib	
   export PATH=${UIMA_HOME}/bin:${PATH}

   export APR_HOME=/usr
   export ICU_HOME=/usr
   export XERCES_HOME=/usr

   export JAVA_HOME=/usr/lib/jvm/java-7-openjdk-amd64/
   export JAVA_INCLUDE=${JAVA_HOME}/include

   export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH}

Doing *ls $UIMA_HOME* should give something like::
  
   bin  config  docs  eclipsePlugins  examples  issuesFixed  lib  LICENSE  NOTICE  README  RELEASE_NOTES.html

You are ready to compile with `catkin_make`

