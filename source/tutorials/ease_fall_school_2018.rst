.. _ease_fall_school_2018:

============================================
RoboSherlock Tutorial: EASE Fall School 2018
============================================

This tutorial assumes some basic understanding of what ROS is and how to use its tools. The instructions here have been tested under Ubuntu 16.04, though since we are using docke it should work on any OS.

-------------
Prerequisites
-------------

Thtoughout the tutorial we will be using a dockerized version of RoboSherlock. Docker is a virtualization tool, sort of like a virtual machine but not quite. For detail see the docker `homepage <https://docker.com>`_ or `wiki <https://en.wikipedia.org/wiki/Docker_(software)>`_

To install docker follow the instructions found here::

    https://docs.docker.com/install/linux/docker-ce/ubuntu

Once docker is installed you can got the Robosherlock image by opening a terminal and executing::

    docker pull robosherlock/rs_interactive
    
On the host machine create a folder that will be shared with the docker container::

    mkdir sandbox
    
To start the a container you will need to execute docker run with a couple of parameters (mainly for port forwarding so the ineractive web interface can work). Alternatively you can simply forward all ports (might conflict with existin mongodb though)::

    docker run -d -p 3000:3000 -p 8080:8080 -p 5555:5555 -p 9090:9090 -v ./sandbox:/home/rs/sandbox --name rs_demo robosherlock/rs_interactive
    
The ``-d`` option tells docker torun the container as a daemon, you will have to wait for a few seconds untill everything launches. Make sure to give the correct path to the ``sandbox`` folder on the host pc. To check that runnning the container was successfull open a browser (Firefox was tested to work, other browsers might have issues), and go to::

    localhost:3000

An xterm should open asking for a username and a password; Create a new file in the sandbox folder and create a file there::
    
    cd ~/sandbox && touch test_file

You should see the file appearing in the sandbox folder of the host machine. 

.. warning:: Changes made to a docker container persist only until the container exists; If you stop and remove the container changes you have made to it will be lost; To make changes permanent you need to commit them to the image; If you stop the container you can restart it using ``docker start rs_demo``. It is recommended to also use the ``-v path_to_host_folder:path_to_docker_folder`` option to share some of the output files of RoboSherlock with the host (e.g. images) for easier visualization.

The docker container contains a full installation of RoboShelrock, with all the dependencies and all of its current capabilities. To ease the use of the xterm in the browser we have installed byobu (a screen variant) that allows openning multiple terminal windows, verticaland horizontal splitting etc. (Check the `cheat sheets <https://www.iconspng.com/images/byobu-cheat-sheet/byobu-cheat-sheet.jpg>`_ for more detail). The instructions that follow are all meant to be executed in the terminal window in a browser. When prompted for a new terminal window, open a new tab in byobu or split the existing one.


----------------------------
Introduction to RoboSherlock
----------------------------

RoboSherlock is a ROS package, and uses ROS to interface with other components of a robotic system. Before you begin let's set up a new ROS workspace. 


Short ROS primer
----------------


In your user home create a folder for the new workspace and initialize it as a catkin worksapce::
    
    mkdir -p demo_ws/src
    cd demo_ws
    catkin init 
    catkin config --extend /homer/rs/rs_ws/devel
   
Build the empty workspace::
    
    catkin build

and source it::

   source /home/rs/demo_ws/devel/setup.bash

Make sure to add it to your ``bashrc`` so that terminals that you open in the future will know about it::

    echo 'source /home/rs/demo_ws/devel/setup.bash' >> ~/.bashrc


The basics of RoboSherlock
--------------------------

 * Task 1: create a ros package of your own (follow the tutorial from :ref:`create_your_rs_catkin_pkg`.

 * Task 2: run a simple pipeline: follow the tutorial :ref:`pipeline`.
 
 * Task 3: how to write your own annotator :ref:`create_your_own_ae`.
 
 * Task 4: log an execution
  
-------------------------------
Adapting capabilities to a task
-------------------------------

 * Let's look at the results from the previous execution;
 * Let's look at an experiment that we ran on the robot
 * export some images
 * train your own classifier and execute


 
Knowledge integration and query answering
-----------------------------------------

 * The task of RoboSherlock is to complete the objects designators sent by the high-level executive; 

Start by launching knowrob and the web server::

    roslaunch rs_run_configs json_prolog_and_rosbridge.launch 
    
This launches the json prolog interface to knowrob and all web frontend for interacting with RoboSherlock; The web frontend is accessible on port 5555::

    localhost:5555

Use the predifined queries to see what is stored in the knowledge base;

Extend the query answering capabilities with new annotators. The annotators have the following pre- and postconditions:

.. note:: Define a set of new annotators such the following conditions are met;
   AnnotatorA takes as input an annotation of type rs.classification.Annotation; 



    
