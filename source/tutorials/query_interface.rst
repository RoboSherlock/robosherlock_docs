.. _query_interface:

============================
Documentation QueryInterface
============================

The two main parts of the QueryInterface are understanding the query, so that a Pipeline can be generated according to the query, and filtering the result of the Pipeline, so only relevant results are returned.

The Query
-----------------------

The query is parsed from the json string using rapidjson. Currently, the following query modes are supported: ::

	detect
	inspect(untested) 

The detection query should have the following structure:

``'{"detect":{"key1":"value1","key2":"value2", ...}}'``

The keys can currently take the following values: ::

	shape, size, color, type, class, cad-model, contains, volume

The functionality of these keys is defined in ``config/filter_config.ini``.

The filter configuration
------------------------

The filtering checks each result of the pipeline according to the filter configuration. If a check fails, the result is not returned. There are a number of checks implemented. 

A filter configuration may look like this: ::

	[contains]
	location=/substance
	check=EQUAL

This tells the filtering, that if the user queries for contains, look for the value in the child "substance" in the json which describes a result object. Since the check is defined as EQUAL, the value of "substance" in the object description should be equal to the value given in the query.

.. note:: The default location is the query key, so here it would be /contains (which needed to be changed). If the location is the same as the query key then location doesn't need to be specified. The default check is EQUAL. This means, that if you query for something where the location is the same as the query key and you want to check EQUAL, it doesn't need to be defined in the configuration file.


In the following section, each possible check is described in detail.

EQUAL
-----

As described above, the EQUAL check compares the given query value with the value at the location specified in the filter configuration.

CLASS
-----

The check CLASS passes, if the value at the specified location is a subclass of the value given in the query.

GEQ
---

GEQ checks, whether the value at the location is greater or equal to the value given in the query.

CONTAINS
--------

The check CONTAINS works on lists. It is required, that the json at the specified location is a list. The check fails, if the list does not contain the value given in the query.

THRESHLIST
----------

Additional Parameters: 

threshold - The threshold which should be applied

keepLower - Whether anything lower should be kept or anything greater or equal

The check THRESHLIST works on lists as well. However, it does not check, whether a value is equal to the queried value. Instead, it applies the threshold from the configuration to a certain object in the list. Which object depends on the query value. As an example: ::

	threshold:0.2

	Query: {"detect":{"color":"red"}}

	Object1:{"colors":["red":0.3,"blue":0.1]}

	Object2:{"colors":["red":0.1,"blue":0.1]}

The check would now look at the red values of Object1 and Object2. In this case, the check would succeed for Object1 but fail for Object2.

CONTAINSEQUAL -Should be merged with CONTAINS
---------------------------------------------

The CONTAINSEQUAL check works the same as CONTAINS, with the exception, that the value in the json representation is in an object which itself is in a list. This means, that the location in the json representation is unknown, since all of the list entries must be checked. An example location could look like this: ::

	location: /pose/2/source

This means, that we are looking at the 2. pose in the list of poses. 
In the configuration, the existance of a list at a certain point is indicated by a *. So the location in the configuration would look like this: ::

	location: /pose/*/source

CONTAINSEQUAL checks all of the objects in a list, and if it finds an object, where the value at the location is equal to the value which was queried for, the check succeeds for that result and it is returned.