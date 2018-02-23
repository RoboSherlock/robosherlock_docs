.. _write_queries:

===================================
Use queries to control the pipeline
===================================

This tutorial assumes that the reader has started RoboSherlock, downloaded the bag file available in the :ref:`first tutorial<pipeline>` and loaded the bagfile into the MongoDB as described in the :ref:`tutorial about logging results<mongodb>`. 

RoboSherlock supports querying for certain characteristics, which it will try to perceive in the scene.
As an example, the user may want to know, whether there are any blue objects in the scene.

Querying can be done using the ROS-Service Interface. The query itself is a json string. Execute the following query, which corresponds to the example above. ::

	rosservice call /query_json "query: '{\"detect\":{\"color\":\"blue\"}}'" 

.. note:: The quotation marks in the json string need to be escaped when using rosservice from the command line.

The result is a list of objects that fit the query. In this case, every object, which contains at least 20% blue color. 

Queries can be composed of multiple desired characteristics. Execute the following query and note, that the result now contains only the flat blue objects. ::

	rosservice call /query_json "query: '{\"detect\":{\"color\":\"blue\", \"shape\":\"flat\"}}'" 


A list of available characteristics can be found here.

Change the query configuration
------------------------------

Some query characteristics like color can be configured to modify the way they are filtered. In the example above the object had to contain 20% blue color in order to be returned as a blue object. These values can be changed in the ``config/filter_config.ini``. Open the file in an editor and find the entry for color. Change the threshold value to 0.4. The next time the color is queried, objects need to have at least 40% of a certain color to fit that characteristic.
