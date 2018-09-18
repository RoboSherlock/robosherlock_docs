.. _create_your_rs_catkin_pkg:

===========================================
Create your own catkin-robosherlock package
===========================================

Let's create a new package called ``rs_tutorial``. Make sure you are in the source folder of a catkin workspace and run::

    rosrun robosherlock rs_create_package rs_tutorial


or if you've added the ``scripts`` folder to your PATH simply run::
    
    rs_create_package rs_tutorial

The script will create a new catkin package that has the structure needed for Robosherlock.::

   'package_name'
    |-descriptors         
       |-analysis_engines -> yaml definitions of aggregate AEs
       |-annotators       -> yaml definitions of primitive AEs
       |-typesystem       -> yaml deginitions of the typesystem
          |-all_types.xml -> typesystem definition
    |-include
       |-package_name     -> include folder
          |-types         -> folder for the auto-generated type implementations
    |-src                 -> code base
    |-package.xml         -> catkin package xml   
    |-CMakeLists.txt      -> CMake file


It edits the *CMakeLists.txt*, sets the CMake variables needed for code generation. finding annotations automatically::

  ################################################################################
  ## Constants for project                                                      ##
  ################################################################################
  set(NAMESPACE package_name)
  set(TYPESYSTEM_CPP_PATH ${PROJECT_SOURCE_DIR}/include/package_name/types)
  set(TYPESYSTEM_XML_PATH ${PROJECT_SOURCE_DIR}/descriptors/typesystem)
  set(ANNOTATOR_PATH      ${PROJECT_SOURCE_DIR}/descriptors/annotators)
  set(ENGINE_PATH         ${PROJECT_SOURCE_DIR}/descriptors/analysis_engines)


  ################################################################################
  ## Update analysis engines, typesystem and include all relevant files         ##
  ################################################################################
  
  ## generate classes from the typesystem xml files
  generate_type_system(robosherlock)
  #find all relevant files
  find_additional_files()

The first part sets the five CMake variables that are in turned used by the scripts that are called in the second part. 
	
	* generate_type_system: checks if we have newly defined types in the xml descriptions and generates the C++ container classes for them
	
You can now add your custom annotators and pipeline analysis engines that can use any component defined in the RoboSherlock core package. If you want ``rs_tutorial`` to depend on other robosherlock packages add them to the ``package.xml`` and ``CmakeLists.txt``.
