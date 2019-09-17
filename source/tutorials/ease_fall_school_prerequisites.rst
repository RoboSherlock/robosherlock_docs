..  _ease_fall_school_prerequisites:

=====================================================
Prerequisietes for EASE Fall School hands on tutorial
=====================================================

Linux is the recommended OS, since the instructions here were tested under Ubuntu 16.04 and Ubuntu 18.04. Although, since you will be using using docker, it should work on any OS.

Throughout the tutorial you will be using a dockerized version of RoboSherlock. Docker is a virtualization tool, sort of like a virtual machine but not quite. For detail see the docker `homepage <https://docker.com>`_ or `wiki <https://en.wikipedia.org/wiki/Docker_(software)>`_

To install docker follow the instructions found here::

    https://docs.docker.com/install/linux/docker-ce/ubuntu
    
Make sure to check the post installation instructions after the installation::

    https://docs.docker.com/install/linux/linux-postinstall/

Specifically ensure that you do not need root access to run a docker container. 

Once docker is installed you can get the RoboSherlock image by opening a terminal and executing::

    docker pull robosherlock/rs_interactive
    
On the host machine create a folder that will be shared with the docker container::

    mkdir sandbox
    
To start the a container you will need to execute docker run with a couple of parameters (mainly port forwarding). Alternatively you can simply forward all ports (might conflict with existing software on your system though)::

    docker run -d -p 3000:3000 -p 8080:8080 -p 5555:5555 -p 9090:9090 -p 8081:8081 -v ./sandbox:/home/rs/sandbox --name rs_demo robosherlock/rs_interactive
    
The ``-d`` option tells docker to run the container as a daemon, you will have to wait for a few seconds until everything launches. Make sure to give the correct path to the ``sandbox`` folder on the host PC. To check that running the container was successful open a browser (Firefox was tested to work, other browsers might have issues), and go to::

    localhost:3000

An xterm should open asking for a username and a password (username: rs; password: rs); To check that the volume has mounted, create a new file in the sandbox folder::
    
    cd ~/sandbox && touch test_file

You should see the file appearing in the sandbox folder of the host machine. 

.. warning:: Changes made to a docker container persist only until the container exists; If you stop and remove the container changes you have made to it will be lost; To make changes permanent you need to commit them to the image; If you stop the container you can restart it using ``docker start rs_demo``.

The docker container contains a full installation of RoboShelrock, with all the dependencies and (almost) all of its current capabilities. To ease the use of the xterm in the browser, byobu (a screen variant) is installed, that allows opening multiple terminal windows, vertical and horizontal splitting etc. (Check the `cheat sheets <https://www.iconspng.com/images/byobu-cheat-sheet/byobu-cheat-sheet.jpg>`_ for more detail). The instructions that follow are all meant to be executed in the terminal window in a browser. When prompted for a new terminal window, open a new tab in your browsed or in byobu.