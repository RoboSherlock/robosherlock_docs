==============================
Write your own Analysis Engine
==============================

This tutorial assumes that the reader has created a robosherlock package, and allready run the pipeline as described in the TODO section.

As descriped in the overview (TODO:link) analysis enignes can be either primitive or aggregate. A primitive analysis engine is called an annotator. In the following the creation of a new primitive analysis engine will be described, followed by creating an aggregat AE that uses is.

.. note:: it is not required that all primitive analysis engines annotate a scene, but for simplicity we call individual experts annotators. E.g. primitive AEs can generate object hypotheses, or have I/O tasks

Create your annotator
---------------------

Since, besides the implementation, it is mandatory in the UIM framework to have meta definitions of every component, we have created a small script that makes creating new components a bit faster. Execute::
  
  rosrun robosherlock rs_new_annotator rs_test MyFirstAnnotator

which will create a new annotator called *MyFirstAnnotator* in the previously created ROS-package *rs_test*. It essentially creates an xml meta file in *descriptors/annotators* and a source file in *./src*. In order to compile it you need to add the following two lines in the CMakeLists.txt::

  rs_add_library(rs_myFirstAnnotator src/MyFirstAnnotator.cpp)
  target_link_libraries(rs_myFirstAnnotator rs_core)

Every component in RoboSherlock is basically a c++ library, that gets loaded during runtime.::
    
	<?xml version="1.0" encoding="UTF-8"?>
	<taeDescription xmlns="http://uima.apache.org/resourceSpecifier">
	  <frameworkImplementation>org.apache.uima.cpp</frameworkImplementation>
	  <primitive>true</primitive>
	  <annotatorImplementationName>rs_myFirstAnnotator</annotatorImplementationName>
	  <analysisEngineMetaData>
	    <name>MyFirstAnnotator</name>
	    <description/>
	    <version>1.0</version>
	    <vendor/>
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
	    <typeSystemDescription>
	      <imports>
	        <import location="../typesystem/typesDsescriptor.xml"/>
	      </imports>
	    </typeSystemDescription>
	    <capabilities>
	    <capability>
	        <inputs/>
	        <outputs/>
	        <languagesSupported>
	          <language>x-unspecified</language>
	        </languagesSupported>
		</capability>
	    </capabilities>
	    <operationalProperties>
	      <modifiesCas>true</modifiesCas>
	      <multipleDeploymentAllowed>true</multipleDeploymentAllowed>
	      <outputsNewCASes>false</outputsNewCASes>
	    </operationalProperties>
	  </analysisEngineMetaData>
	</taeDescription>
   
Now in the `src` folder create `MyFirstAnnotator.cpp` containing the following code::
    
	#include <uima/api.hpp>

	#include <pcl/point_types.h>
	#include <iai_rs/types/all_types.h>
	//RS
	#include <iai_rs/scene_cas.h>
	#include <iai_rs/util/time.h>
	#include <iai_rs/DrawingAnnotator.h>

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
	
	  TyErrorId typeSystemInit(TypeSystem const &type_system)
	  {
	    outInfo("typeSystemInit");
	    return UIMA_ERR_NONE;
	  }
	
	  TyErrorId destroy()
	  {
	    outInfo("destroy");
	    return UIMA_ERR_NONE;
	  }
	
	  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec)
	  {
	    outInfo("process start");
	    iai_rs::util::StopWatch clock;
	    iai_rs::SceneCas cas(tcas);
	    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
	
	    outInfo("Test param =  " << test_param);
	
	    cas.getPointCloud(*cloud_ptr);
	
	    outInfo("Cloud size: " << cloud_ptr->points.size());
	    outInfo("took: " << clock.getTime() << " ms.");
	    return UIMA_ERR_NONE;
	  }
	
	};
	
	// This macro exports an entry point that is used to create the annotator.
	MAKE_AE(MyFirstAnnotator)

Now compile it with catkin_make. Let us now go through what we have just done step by step:

To be continued....
