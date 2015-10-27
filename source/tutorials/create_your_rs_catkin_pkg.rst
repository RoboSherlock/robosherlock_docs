.. _create_your_rs_catkin_pkg:
===========================================
Create your own catkin-robosherlock package
===========================================

Make sure you are in the source folder of a catkin workspace and run::

    rosrun robosherlock rs_create_package <package_name>


or if you've added the ``scripts`` folder to your PATH simply run::
    
    rs_create_package <package_name>

The script will create a new catkin package that has the structure needed for Robosherlock.::

   'package_name'
    |-descriptors         
       |-analysis_engines -> xml definitions of aggregate AEs
       |-annotators       -> xml definitions of primitive AEs
       |-typesystem       -> xml deginitions of the typesystem
          \-all_types.xml -> typesystem definition
    |-include
	   |-package_name     -> include folder
	      |-types         -> folder for the auto-generated type implementations
    |-src                 -> code base
    |-package.xml         -> catkin package xml   
    |-CMakeLists.txt      -> CMake file


It edits the *CMakeLists.txt*, seting the CMake variables needed for generating code. finding annotations automatically::

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
  ##Update xml list of annotators inside analysis engines
  update_analysis_engines(robosherlock)
  ## generate classes from the typesystem xml files
  generate_type_system(robosherlock)
  #find all relevant files
  find_additional_files()


The first part sets the five CMake variables that are in turned used by the scripts that are called in the second part. 
	
	* update_analysis_engines: checks if there are new annotators defined in the current package and any package that depends on RoboSherlock and is given as a parameter and adds the relative path of these to the analysis engines defined in our current package
	* generate_type_system: checks if we have newly defined types in the xml descriptions and generates the C++ container classes for them
	
You can now add your custom annotators and pipeline analysis engines that can use any component defined in the RoboSherlock core package. To see the effect of the scripts, try copying over an analysis engine from the robosherlock package (e.g. the demo.xml) to *<package_name>/descriptors/analysis_engines*. After compilations notice how the paths of the components have changed.
