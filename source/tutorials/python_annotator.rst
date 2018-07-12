.. _gsoc_python_annotator:

==================================
Use Deep learning Python Annotator
==================================

This tutorial is based on Google Summer of Code(GSoC) 2018 program by Shingo Kitagawa.
The GSoC project and proposal page is `here <gsoc2018pythonannotar>`_.

The things below is done:

* Run Python code with boost:python and boost:numpy from C++ annotator.

* Implement Faster-RCNN Detection annotator with Python library Chainer.

* Implement SSD Detection annotator with Python library Chainer.

* Implement FCIS Instace Segmentation annotator with Python library Chainer.

.. image:: ../imgs/faster_rcnn_robosherlock.png

.. image:: ../imgs/fcis_robosherlock.png

.. _gsoc2018pythonannotar: https://summerofcode.withgoogle.com/dashboard/project/4651529278062592/overview/

Requirement
-----------
* boost::python
* boost::numpy
* numpy
* chainer
* chainercv

How to run demo
---------------

Faster-RCNN
~~~~~~~~~~~

.. code-block:: bash

   git clone https://github.com/knorth55/rs_test.git
   cd rs_test
   catkin bt
   roslaunch robosherlock rs.launch ae:=faster_test
   rosbag play --loop --clock test.bag

SSD
~~~

.. code-block:: bash

   git clone https://github.com/knorth55/rs_test.git
   cd rs_test
   catkin bt
   roslaunch robosherlock rs.launch ae:=ssd_test
   rosbag play --loop --clock test.bag

FCIS
~~~~

.. code-block:: bash

   git clone https://github.com/knorth55/rs_test.git
   cd rs_test
   catkin bt
   roslaunch robosherlock rs.launch ae:=fcis_test
   rosbag play --loop --clock test.bag
