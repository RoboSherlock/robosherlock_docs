.. _add-remove-cas::

==============================
Reading and writing in the CAS
==============================


In **uimacpp**, internally everything is represented as a subclass of the class FeatureStructure. Much like the way in Java everything is an Object. While this is needed in order to maintain compatibility with the java implementation of the uima library (one could easily run pipelines defined in RoboSherlock through the java tools that are offered by the UIM library), it also introduces an extra layer of complexity that is not desired when writing code. Since we can only store data that has a type defined in the typesystem, and these are feature Structures we need to convert them FFS.(uuugly). 
