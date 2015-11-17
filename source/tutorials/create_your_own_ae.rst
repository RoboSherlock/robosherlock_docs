.. _create_your_own_ae:

==============================
Write your own Analysis Engine
==============================

This tutorial assumes that the reader has created a robosherlock package, and allready run the pipeline as described in the TODO section.

As descriped in the overview (TODO:link) analysis enignes can be either primitive or aggregate. A primitive analysis engine is called an annotator. In the following the creation of a new primitive analysis engine will be described, followed by creating an aggregat AE that uses is.

.. note:: it is not required that all primitive analysis engines annotate a scene, but for simplicity we call individual experts annotators. E.g. primitive AEs can generate object hypotheses, or have I/O tasks

Create your annotator
---------------------

Besides the implementation, it is mandatory in the UIM framework to have meta definitions of every component. A small script is available that makes creating new components a bit faster. Execute::
  
  rosrun robosherlock rs_new_annotator rs_test MyFirstAnnotator

which will create a new annotator called *MyFirstAnnotator* in the previously created ROS-package *rs_test*. It creates an xml meta file in *descriptors/annotators* and a source file in *./src*. In order to compile it you need to add the following two lines in the CMakeLists.txt::

  rs_add_library(rs_myFirstAnnotator src/MyFirstAnnotator.cpp)
  target_link_libraries(rs_myFirstAnnotator rs_core)

Every component in RoboSherlock is a  C++ library, that gets loaded during runtime. The implementation consists of a cpp file and an xml descriptor.

The xml descriptor
------------------

The first important part in the descriptor is the tag that tells the system where the annotator is implemented:: 

	  <primitive>true</primitive>
	  <annotatorImplementationName>rs_myFirstAnnotator</annotatorImplementationName>
	  
The value here is the exact name of the library file that is being generated during compilation. Setting the primitive tag to true signals the system that the descriptor is for single module (setting this to true would make it an aggregate analysis engine, one that we use for defining pipelines).

This is followed by meta data of the annotator (name, version a description etc)

  - name of the annotator, that is used to reference it from a pipeline::
	
		<name>MyFirstAnnotator</name>
  
  - configuration parameters (declaration is separate from parameter settings, since it is not mandatory to define values here. They can be set to optional and defined in the analysis engines)::
  
	    <configurationParameters>
	      <configurationParameter>
	        <name>test_param</name>
	        <type>Float</type>
	        <multiValued>false</multiValued>
	        <mandatory>false</mandatory>
	      </configurationParameter>
	    </configurationParameters>
	    <configurationParameterSettings>
	      <nameValuePair>
	        <name>test_param</name>
	        <value>
	          <float>0.01</float>
	        </value>
	      </nameValuePair>
	    </configurationParameterSettings>
	    
  - path to the type-system(make sure the file actually exists)::
	    
	    <typeSystemDescription>
	      <imports>
	        <import location="../typesystem/all_types.xml"/>
	      </imports>
	    </typeSystemDescription>
  
  - capabilities of the annotator in terms of I/O as defined in the type-system::
	    
   <capabilities>
      <capability>
            <inputs/>
            <outputs/>
          </capability>
        </capabilities>
   
The cpp implementation
----------------------

`MyFirstAnnotator.cpp` was generated in the ``src`` folder::
    
	#include <uima/api.hpp>

	#include <pcl/point_types.h>
	//RS
	#include <rs/types/all_types.h>
	#include <rs/scene_cas.h>
	#include <rs/utils/time.h>

	using namespace uima;

	class MyFirstAnnotator : public Annotator
	{
	private:
	  float test_param;

	public:

	  TyErrorId initialize(AnnotatorContext &ctx)
	  {
	    outInfo("initialize");
	    ctx.extractValue("test_param", test_param);
	    return UIMA_ERR_NONE;
	  }
	
	  TyErrorId destroy()
	  {
	    outInfo("destroy");
	    return UIMA_ERR_NONE;
	  }
	
	  TyErrorId process(CAS &tcas, ResultSpecification const &res_spec)
	  {
	    outInfo("process start");
	    rs::StopWatch clock;
	    rs::SceneCas cas(tcas);
	    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
	    outInfo("Test param =  " << test_param);
	    cas.get(VIEW_CLOUD,*cloud_ptr);
	
	    outInfo("Cloud size: " << cloud_ptr->points.size());
	    outInfo("took: " << clock.getTime() << " ms.");
	    return UIMA_ERR_NONE;
	  }
	};
	
	// This macro exports an entry point that is used to create the annotator.
	MAKE_AE(MyFirstAnnotator)

Implementation of an annotator extends the ``Annotator`` class of the uimacpp library. ``Annotator`` has several virtual methods defined out of which we are overriding the ``initialize``, ``destroy`` and ``process`` functions. Since annotators get compiled into runtime libraries they must end with the ``MAKE_AE(<AnnotName>)`` macro, that exports the entry point.

The three methods that we overwrite implement the functionalities of the annotator:

	- ``initialize`` : gets called in the constructor of the class. Has the same functionalities as a constructor. We can read in the parameters defined in the xml descriptor here (in the tutorial code this is *test_param*).
	- ``destroy`` :  It's like a destructor of a class, e.g. deallocate memory, if needed. 
	- ``process`` :  this is where all the processing code goes. In the tutorial we convert the cas to the SceneCas, get the point cloud that we stored in it and display it's size

.. note:: ``SceneCas`` is a wrapper for the uima::CAS class from uimacpp for conveniently setting and getting data. 


You can now compile it with catkin_make.

Add it to an AE and run
-----------------------

In the previous  :ref:`tutorial <create_your_rs_catkin_pkg>` we copied over the demo.xml to our poroject. Start by renaming it to something like *my_demo.xml* so the naming does not collide with the one in the robosherlock package. Open my_demo.xml and add your new annotator to the pipeline by adding a new *<node>* tag in the fixed flow:

.. note:: Notice that during compilation MyFirstAnnotator was added to the  *delegateAnalysisEngineSpecifiers*

Your fixed flow should look something like this now: 

.. code-block:: xml
   :lineno-start: 133 
   :emphasize-lines: 4
   
   <fixedFlow>
   <node>CollectionReader</node>
   <node>ImagePreprocessor</node>
   <node>MyFirstAnnotator</node>
   <node>PointCloudFilter</node>
   <node>NormalEstimator</node>
   <node>PlaneAnnotator</node>
   <node>ImageSegmentationAnnotator</node>
   <node>PointCloudClusterExtractor</node>
   <node>ClusterMerger</node>
   <node>ResultAdvertiser</node>
   </fixedFlow>
   
Run the pipeline as described in :doc:`pipeline`. Look at the output in your terminal. There should be an output with the value of the test parameter, and the number of points in the point cloud. 

.. note:: It is recommended to  create you own launch file in the current package. Notice that you have to change the arguments of the ros node in the launch file in order to execute your new pipeline( from demo to my_demo)

.. warning:: The annotators execute in the order they are defined in the fixed flow. Since the demo annotator accesses point clouds it needs to be put after the ImagePreprocessor component, since this is the module that creates the point cloud from the depth and rgb images. 

The output in the terminal should look like this::

   MyFirstAnnotator.cpp(40)[process] process start
   MyFirstAnnotator.cpp(44)[process] Test param =  0.01
   MyFirstAnnotator.cpp(47)[process] Cloud size: 307200
   MyFirstAnnotator.cpp(48)[process] took: 2.37502 ms.

