.. _installation:


=========================
Installation instructions
=========================


The recommended operating system for building RoboSherlock is Ubuntu 20.04 or Ubuntu 18.04. *RoboSherlock* comes as a ROS stack, so you will need to install **ROS Noetic**, **ROS Melodic** or **ROS Kinetic**. Installation instructions can be found on the ROS homepage_ and setting up  a catkin workspace is described here_.


.. _homepage: http://wiki.ros.org/ROS/Installation
.. _here: http://wiki.ros.org/catkin/Tutorials/create_a_workspace

The following instructions are valid for the current **noetic**, **melodic or kinetic branches** of the core package of RoboSherlock which you can get from the GitHub page. For previous versions see below. Go to your catkin workspace's **src** folder and check out the repository.

For **ROS Noetic** run::

    git clone https://github.com/RoboSherlock/robosherlock -b noetic --recursive
    git clone https://github.com/ros-perception/image_transport_plugins.git -b noetic-devel # fixes problems with old depth image formats

If you are using **ROS Melodic** run::

    git clone https://github.com/RoboSherlock/robosherlock -b melodic --recursive

For **ROS Kinetic** run:: 

    git clone https://github.com/RoboSherlock/robosherlock -b kinetic --recursive

Install the dependencies::

   cd robosherlock
   rosdep install --from-path . --ignore-src 

Set up Bash
-----------

It is recommended to add the [..]/robosherlock/scripts/ folder to your PATH. This way you can easily access some convenience scripts, for e.g. creating a new annotator, or a new ROS package that depends on RoboSherlock.

Compilation
-----------

We recommend using `catkin tools` to compile your workspace but you  can use `catkin_make` as well.

.. note:: Catkin tries to max out the amount of compilation processes based on the available CPU cores. In general this is a good thing, but you might run out of memory if you have a lot of CPU ores but only a few Gigabytes of RAM. If you run out of memory during compilation, try to reduce the amount of build process with `catkin build -jX` where X is an integer defining the number of parallel build processes.

Check out :ref:`pipeline` for details about how to run a small demo.


Installing and Running the query-answering
------------------------------------------

In order to use the query-answering capabilities of RoboSherlock to the fulles installing Knowrob is recommended. While basic query answering capabilities are working with the internal SWI-PROLOG engine, KnowRob
offers more capabilities:

Knowrob: Follow the official installation guide for the Knowrob Installation, which can be found on their installation page_.

.. note:: the latest versions of KnowRob are currently not compatible with RoboSherlock, due to a conflicting SWI-PROLOG versions.

.. _page: http://www.knowrob.org/installation


Check out :ref:`write_queries` for details about how to use the query-answering.


Contributing
------------

The noetic, melodic and kinetic branches are there only for releasing and mainting a stable version. To contribute to the project use the master branch. Changes made to the master will periodically be merged into the release branches. Fork the repository on github, create a new branch based on master, make your changes and open a pull request.  Direct changes to the release branches will only be merged if they are specific to a that release.

To compile the master branch you might need to install dependencies manually, since a common package.xml can not be maintained for both distributions.


Previous versions
-----------------

:ref:`install_v0.1`

:ref:`install_v1.0`
