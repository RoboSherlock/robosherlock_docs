.. _create_your_rs_catkin_pkg:
===========================================
Create your own catkin-robosherlock package
===========================================

Make sure you are in the source folder of the same catkin workspace as the one of the core RoboSherlock package and run::

    rosrun iai_rs_cpp rs_create_package.sh <package_name>

The script will create a new catkin package that has the structure needed for Robosherlock.::

   'package_name'
    |-descriptors         
       |-analysis_engines -> xml definitions of aggregate AEs
       |-annotators       -> xml definitions of primitive AEs
       |-typesystem       -> xml deginitions of the typesystem
          \-allTypes.xml  -> typesystem definition pointing to the typesystem defined in the core package
    |-src                 -> code base
    |-package.xml         -> catkin package xml   
    |CMakeLists.txt       -> CMake file

It edits the *CMakeLists.txt* adding the line::
    
    update_analysis_engines(descriptors/analysis_engines descriptors/annotators)

which ensures that during compilation the right paths are gonna get set in the aggregate analysis engines(paths for the xml files of the annotators).
Furthermore it creates an initial descriptor for the typesystem , *typeDescriptors.xml* which imports the exisitng typesystem from the core ``iai_rs_cpp`` package. 
**IMPORTANT:** Unfortunatelly at the moment the automatic code generation for the typesystem only works if we extend it in the core package. If new types are added to the current package the container classes for the type need to be implemented manually. 

You are now ready to create you first annotator and an analysis engine that uses it.
